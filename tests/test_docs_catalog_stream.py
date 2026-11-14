"""The signed manual catalog is parsed one page at a time.

As one cJSON tree the catalog needs more than 1 MB of PSRAM, which a board
with 2 MB of PSRAM does not have free, so `help update`, the boot-time check of
a downloaded manual and every Help page load parse its pages one at a time.
This test runs the production catalog code on the catalog that
scripts/generate_manual.py builds from doc/manual.
"""

from pathlib import Path
import json
import re
import subprocess
import tempfile
import unittest

from test_foreground_presenter import production_functions

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "src/services/solar_os_docs.c"
CJSON = sorted(Path.home().glob(".platformio/packages/framework-espidf*/components/json/cJSON"))

FUNCTIONS = (
    "docs_package_enabled",
    "docs_catalog_page_applies",
    "docs_revision_valid",
    "docs_id_valid",
    "docs_page_metadata",
    "docs_manual_page_bytes",
    "docs_validate_catalog_page",
    "docs_validate_catalog_pages",
    "docs_parse_catalog",
    "docs_manual_index_free",
    "docs_manual_copy_string",
    "docs_manual_copy_string_array",
    "docs_manual_index_pack",
    "docs_manual_index_page",
    "docs_build_manual_index",
    "docs_find_catalog_page",
)
TYPES = (
    "docs_catalog_t",
    "docs_pages_t",
    "docs_manual_index_t",
    "docs_page_ids_t",
    "docs_index_build_t",
    "docs_page_lookup_t",
)

HARNESS = r'''
#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "solar_os_docs.h"
#include "solar_os_json.h"
#include "solar_os_memory.h"

#define SOLAR_OS_CRYPTO_SHA256_HEX_LEN 65

static size_t live_bytes, peak_bytes;

void *solar_os_memory_alloc(size_t size, solar_os_memory_class_t memory_class, const char *tag)
{
    (void)memory_class;
    (void)tag;
    max_align_t *block = malloc(sizeof(max_align_t) + size);
    if (block == NULL) return NULL;
    *(size_t *)block = size;
    live_bytes += size;
    if (live_bytes > peak_bytes) peak_bytes = live_bytes;
    return block + 1;
}

void *solar_os_memory_calloc(size_t count, size_t size, solar_os_memory_class_t memory_class,
                             const char *tag)
{
    void *pointer = solar_os_memory_alloc(count * size, memory_class, tag);
    if (pointer != NULL) memset(pointer, 0, count * size);
    return pointer;
}

void solar_os_memory_free(void *ptr)
{
    if (ptr == NULL) return;
    max_align_t *block = (max_align_t *)ptr - 1;
    live_bytes -= *(size_t *)block;
    free(block);
}

static bool solar_os_crypto_sha256_hex_is_valid(const char *hex)
{
    size_t length = 0;
    for (; hex[length] != '\0'; length++) {
        if (!isxdigit((unsigned char)hex[length]) || isupper((unsigned char)hex[length])) return false;
    }
    return length == 64;
}

static const solar_os_manual_page_t embedded_pages[] = {
    {.id = "help", .body = "embedded body", .markdown = "embedded markdown"},
};

size_t solar_os_manual_embedded_count(void) { return 1; }

const solar_os_manual_page_t *solar_os_manual_embedded_get(size_t index)
{
    return index < 1 ? &embedded_pages[index] : NULL;
}
'''

MAIN = r'''
static void print_string(const char *text)
{
    if (text == NULL) {
        fputs("null", stdout);
        return;
    }
    putchar('"');
    for (const unsigned char *c = (const unsigned char *)text; *c != '\0'; c++) {
        if (*c == '"' || *c == '\\') printf("\\%c", *c);
        else if (*c < 0x20) printf("\\u%04x", *c);
        else putchar(*c);
    }
    putchar('"');
}

int main(int argc, char **argv)
{
    assert(argc == 3);
    FILE *file = fopen(argv[1], "rb");
    assert(file != NULL);
    static char catalog[1024 * 1024];
    const size_t length = fread(catalog, 1, sizeof(catalog) - 1, file);
    fclose(file);
    catalog[length] = '\0';

    docs_pages_t pages;
    docs_catalog_t info;
    esp_err_t err = docs_parse_catalog(catalog, length, &pages, &info, true);
    docs_manual_index_t *index = NULL;
    if (err == ESP_OK) err = docs_build_manual_index(&pages, &info, &index);
    if (strcmp(argv[2], "reject") == 0) {
        printf("%d\n", (int)err);
        docs_manual_index_free(index);
        assert(live_bytes == 0);
        return 0;
    }
    assert(err == ESP_OK);
    assert(pages.json > catalog && pages.json + pages.len < catalog + length);
    assert(index->blob_used == index->blob_size);
    const size_t build_peak = peak_bytes;

    docs_page_lookup_t lookup = {.id = "help"};
    assert(solar_os_json_array_for_each(pages.json, pages.len, docs_find_catalog_page,
                                        &lookup, NULL) == ESP_OK);
    assert(lookup.found && strcmp(lookup.relative, "manual/help.md") == 0);
    docs_page_lookup_t missing = {.id = "no-such-page"};
    assert(solar_os_json_array_for_each(pages.json, pages.len, docs_find_catalog_page,
                                        &missing, NULL) == ESP_OK);
    assert(!missing.found);

    docs_manual_index_pack(&index);
    assert(index->packed);
    printf("{\"page_count\":%zu,\"index_bytes\":%zu,\"build_peak\":%zu,\"pages\":[",
           info.page_count, info.index_bytes, build_peak);
    for (size_t i = 0; i < index->count; i++) {
        const solar_os_manual_page_t *page = &index->pages[i];
        const char *fields[] = {page->id, page->title, page->section, page->section_title,
                                page->summary, page->aliases, page->keywords, page->contract,
                                page->body};
        fputs(i ? ",[" : "[", stdout);
        for (size_t f = 0; f < sizeof(fields) / sizeof(fields[0]); f++) {
            if (f) putchar(',');
            print_string(fields[f]);
        }
        putchar(']');
    }
    puts("]}");
    docs_manual_index_free(index);
    assert(live_bytes == 0);
    return 0;
}
'''


def type_definition(source, name):
    match = re.search(r"^typedef (?:struct \{[^}]*\}|char) " + name + r"[^;]*;", source, re.M)
    if match is None:
        raise AssertionError(name)
    return match.group(0)


@unittest.skipUnless(CJSON, "needs the ESP-IDF cJSON sources from PlatformIO")
class DocsCatalogStreamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        cls.catalog_path = tmp / "catalog.json"
        version = (ROOT / "version.txt").read_text().strip()
        subprocess.run(
            ["python3", str(ROOT / "scripts/generate_manual.py"),
             "--input", str(ROOT / "doc/manual"),
             "--packages", str(ROOT / "packages/solar_os_packages.toml"),
             "--version", version,
             "--catalog-output", str(cls.catalog_path),
             "--archive-output", str(tmp / "manual.zip")],
            check=True)
        cls.catalog = json.loads(cls.catalog_path.read_text())

        # Enable some packages, so the index holds some gated pages and drops others.
        gated = sorted({p["packages_any"][0] for p in cls.catalog["pages"] if p["packages_any"]})
        cls.packages = set(gated[::3])

        source = DOCS.read_text()
        defines = "\n".join(re.findall(r"^#define DOCS_\w+ .*$", source, re.M))
        types = "\n".join(type_definition(source, name) for name in ("docs_page_id_t",) + TYPES)
        fields = re.search(r"^static const char \*const docs_manual_string_fields\[\] = \{[^}]*\};",
                           source, re.M).group(0)
        code = (HARNESS
                + f'#define SOLAR_OS_VERSION "{version}"\n'
                + f'#define SOLAR_OS_PACKAGE_ID_LIST "{" ".join(sorted(cls.packages))}"\n'
                + defines + "\n" + types + "\n" + fields + "\n"
                + production_functions(source, FUNCTIONS) + MAIN)
        cls.binary = tmp / "catalog_stream"
        cjson = CJSON[0]
        subprocess.run(
            ["cc", "-x", "c", "-std=c11", "-Wall", "-Wextra", "-Werror",
             "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
             "-D_GNU_SOURCE", "-include", str(ROOT / "tests/host/compat.h"),
             "-I", str(ROOT / "tests/host"), "-I", str(ROOT / "src"),
             "-I", str(ROOT / "src/services"), "-I", str(cjson),
             "-o", str(cls.binary), "-",
             str(ROOT / "src/services/solar_os_json.c"), str(cjson / "cJSON.c"), "-lm"],
            input=code, text=True, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_catalog(self, catalog_path, mode):
        result = subprocess.run([str(self.binary), str(catalog_path), mode],
                                check=True, capture_output=True, encoding="utf-8")
        return result.stdout

    def test_index_matches_the_catalog_and_is_sized_exactly(self):
        dump = json.loads(self.run_catalog(self.catalog_path, "dump"))
        pages = self.catalog["pages"]
        expected = [p for p in pages
                    if not p["packages_any"] or set(p["packages_any"]) & self.packages]
        self.assertGreater(len(expected), 0)
        self.assertLess(len(expected), len(pages))
        self.assertEqual(dump["page_count"], len(pages))
        self.assertEqual([row[0] for row in dump["pages"]], [p["id"] for p in expected])
        for row, page in zip(dump["pages"], expected):
            self.assertEqual(row[1:8], [page["title"], page["section"], page["section_title"],
                                        page["summary"], "\n".join(page["aliases"]),
                                        page["keywords"], page["reference"]])
            self.assertEqual(row[8], "embedded body" if page["id"] == "help" else None)
        text = sum(len(page[key].encode()) + 1 for page in expected
                   for key in ("id", "title", "section", "section_title", "summary",
                               "keywords", "reference"))
        aliases = sum(len("\n".join(page["aliases"]).encode()) + 1 for page in expected)
        self.assertEqual(dump["index_bytes"], text + aliases)
        # The catalog text is the caller's; parsing and building hold only the
        # index and one page at a time.
        self.assertLess(dump["build_peak"], dump["index_bytes"] + 64 * 1024)

    def rejects(self, catalog):
        path = Path(self.tmp.name, "tampered.json")
        path.write_text(catalog if isinstance(catalog, str)
                        else json.dumps(catalog, separators=(",", ":")))
        self.assertNotEqual(self.run_catalog(path, "reject").strip(), "0")

    def tampered(self):
        return json.loads(json.dumps(self.catalog))

    def test_rejects_duplicate_ids(self):
        catalog = self.tampered()
        catalog["pages"][1]["id"] = catalog["pages"][0]["id"]
        catalog["pages"][1]["path"] = catalog["pages"][0]["path"]
        self.rejects(catalog)

    def test_rejects_a_missing_index_field(self):
        catalog = self.tampered()
        page = next(p for p in catalog["pages"] if not p["packages_any"])
        del page["title"]
        self.rejects(catalog)

    def test_rejects_a_catalog_without_an_embedded_topic(self):
        catalog = self.tampered()
        catalog["pages"] = [p for p in catalog["pages"] if p["id"] != "help"]
        self.rejects(catalog)

    def test_rejects_malformed_page_arrays(self):
        text = self.catalog_path.read_text()
        start = text.index('"pages":[') + len('"pages":[')
        self.rejects(text[:start] + "," + text[start:])
        self.rejects(text[:start] + "1," + text[start:])
        self.rejects(text.replace("}],", "},]", 1) if "}]," in text
                     else text[:-3] + ",]}\n")
        self.rejects(text[:start + 100])

    def test_accepts_the_unmodified_catalog(self):
        self.assertEqual(self.run_catalog(self.catalog_path, "reject").strip(), "0")


if __name__ == "__main__":
    unittest.main()
