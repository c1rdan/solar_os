#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "solar_os_json.h"
#include "solar_os_memory.h"

/* Counts the bytes the JSON layer holds, so the test can check that walking
 * an array keeps only one element's tree at a time. */
static size_t live_bytes;
static size_t peak_bytes;

void *solar_os_memory_alloc(size_t size,
                            solar_os_memory_class_t memory_class,
                            const char *tag)
{
    (void)memory_class;
    (void)tag;
    max_align_t *block = malloc(sizeof(max_align_t) + size);
    if (block == NULL) {
        return NULL;
    }
    *(size_t *)block = size;
    live_bytes += size;
    if (live_bytes > peak_bytes) {
        peak_bytes = live_bytes;
    }
    return block + 1;
}

void solar_os_memory_free(void *ptr)
{
    if (ptr == NULL) {
        return;
    }
    max_align_t *block = (max_align_t *)ptr - 1;
    live_bytes -= *(size_t *)block;
    free(block);
}

typedef struct {
    char ids[8][16];
    size_t visited;
    size_t stop_at;
    bool fail_at_second;
} visit_state_t;

static esp_err_t record_id(const solar_os_json_value_t *element,
                           size_t index,
                           bool *stop,
                           void *user)
{
    visit_state_t *state = user;
    assert(index == state->visited);
    if (state->fail_at_second && index == 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    char *id = state->ids[index];
    if (solar_os_json_get_path_string(element, "id", id, sizeof(state->ids[0])) != ESP_OK) {
        strcpy(id, "-");
    }
    state->visited++;
    *stop = state->stop_at != 0 && state->visited == state->stop_at;
    return ESP_OK;
}

static esp_err_t split(const char *json,
                       solar_os_json_doc_t **doc,
                       const char **array,
                       size_t *array_len)
{
    return solar_os_json_parse_object_except_array(json, strlen(json), "pages",
                                                   doc, array, array_len);
}

static void test_split_and_walk(void)
{
    const char *json =
        " { \"a\" : 1 , \"pages\" : [ {\"id\":\"x\"} ,"
        "{\"id\":\"y\",\"n\":[1,{\"z\":\"]}\\\"\"}]} ] ,\"b\":\"t\\\"e}\" } ";
    solar_os_json_doc_t *doc = NULL;
    const char *array = NULL;
    size_t array_len = 0;
    assert(split(json, &doc, &array, &array_len) == ESP_OK);
    assert(strncmp(array, "[ {\"id\":\"x\"}", 12) == 0);
    assert(array[array_len - 1] == ']' && array[array_len] == ' ');

    const solar_os_json_value_t *root = solar_os_json_root(doc);
    uint32_t a = 0;
    char b[8];
    assert(solar_os_json_get_path_uint32(root, "a", &a) == ESP_OK && a == 1);
    assert(solar_os_json_get_path_string(root, "b", b, sizeof(b)) == ESP_OK);
    assert(strcmp(b, "t\"e}") == 0);
    assert(solar_os_json_object_get(root, "pages") == NULL);
    solar_os_json_free(doc);

    size_t count = 0;
    assert(solar_os_json_array_for_each(array, array_len, NULL, NULL, &count) == ESP_OK);
    assert(count == 2);
    visit_state_t state = {0};
    assert(solar_os_json_array_for_each(array, array_len, record_id, &state, &count) == ESP_OK);
    assert(count == 2 && state.visited == 2);
    assert(strcmp(state.ids[0], "x") == 0 && strcmp(state.ids[1], "y") == 0);

    state = (visit_state_t){.stop_at = 1};
    assert(solar_os_json_array_for_each(array, array_len, record_id, &state, &count) == ESP_OK);
    assert(count == 1 && state.visited == 1);

    state = (visit_state_t){.fail_at_second = true};
    assert(solar_os_json_array_for_each(array, array_len, record_id, &state, &count) ==
           ESP_ERR_INVALID_RESPONSE);
    assert(count == 0);
}

static void test_empty_and_scalar_arrays(void)
{
    solar_os_json_doc_t *doc = NULL;
    const char *array = NULL;
    size_t array_len = 0;
    assert(split("{\"pages\":[]}", &doc, &array, &array_len) == ESP_OK);
    size_t count = 7;
    visit_state_t state = {0};
    assert(solar_os_json_array_for_each(array, array_len, record_id, &state, &count) == ESP_OK);
    assert(count == 0 && state.visited == 0);
    solar_os_json_free(doc);

    assert(split("{\"pages\":[ 1, -2.5e3 ,true,null,\"s\" ]}", &doc, &array, &array_len) ==
           ESP_OK);
    assert(solar_os_json_array_for_each(array, array_len, NULL, NULL, &count) == ESP_OK);
    assert(count == 5);
    assert(solar_os_json_array_for_each(array, array_len, record_id, &state, &count) == ESP_OK);
    assert(count == 5);
    solar_os_json_free(doc);
}

static void test_rejects_malformed_documents(void)
{
    solar_os_json_doc_t *doc = NULL;
    const char *array = NULL;
    size_t array_len = 0;
    assert(split("{\"a\":1}", &doc, &array, &array_len) == ESP_ERR_NOT_FOUND);
    assert(split("{\"pages\":{}}", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"pages\":[],\"pages\":[]}", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"pages\":[],\"pag\\u0065s\":[]}", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"a\":,\"pages\":[]}", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"pages\":[] \"a\":1}", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"pages\":[1,2", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("{\"pages\":[\"]", &doc, &array, &array_len) == ESP_FAIL);
    assert(split("[\"pages\"]", &doc, &array, &array_len) == ESP_FAIL);
    assert(doc == NULL && array == NULL && array_len == 0);

    static const char *const bad_arrays[] = {
        "[1,]", "[1 2]", "[,1]", "[{\"id\":}]", "[1] x", "{}", "[\"a]", "[",
    };
    for (size_t i = 0; i < sizeof(bad_arrays) / sizeof(bad_arrays[0]); i++) {
        visit_state_t state = {0};
        assert(solar_os_json_array_for_each(bad_arrays[i], strlen(bad_arrays[i]),
                                            record_id, &state, NULL) == ESP_FAIL);
    }
    /* Counting only finds element boundaries; parsing checks the elements. */
    size_t count = 0;
    assert(solar_os_json_array_for_each("[{\"id\":}]", 9, NULL, NULL, &count) == ESP_OK);
    assert(count == 1);
    assert(live_bytes == 0);
}

static char *read_file(const char *path, size_t *len)
{
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    assert(fseek(file, 0, SEEK_END) == 0);
    const long size = ftell(file);
    assert(size > 0);
    rewind(file);
    char *data = malloc((size_t)size + 1);
    assert(data != NULL && fread(data, 1, (size_t)size, file) == (size_t)size);
    fclose(file);
    data[size] = '\0';
    *len = (size_t)size;
    return data;
}

typedef struct {
    size_t pages;
    size_t peak_from;
} catalog_walk_t;

static esp_err_t check_catalog_page(const solar_os_json_value_t *page,
                                    size_t index,
                                    bool *stop,
                                    void *user)
{
    (void)index;
    (void)stop;
    catalog_walk_t *walk = user;
    char id[64];
    assert(solar_os_json_get_path_string(page, "id", id, sizeof(id)) == ESP_OK);
    assert(solar_os_json_string_value(solar_os_json_object_get(page, "reference")) != NULL);
    assert(solar_os_json_string_value(solar_os_json_object_get(page, "size")) == NULL);
    walk->pages++;
    return ESP_OK;
}

/* The signed manual catalog: walking its pages must hold one page at a time
 * instead of the whole tree. */
static void test_manual_catalog(const char *path)
{
    size_t len = 0;
    char *catalog = read_file(path, &len);

    peak_bytes = live_bytes = 0;
    solar_os_json_doc_t *tree = NULL;
    assert(solar_os_json_parse(catalog, len, &tree) == ESP_OK);
    const size_t tree_bytes = peak_bytes;
    const size_t page_total =
        solar_os_json_array_size(solar_os_json_object_get(solar_os_json_root(tree), "pages"));
    solar_os_json_free(tree);
    assert(live_bytes == 0);

    peak_bytes = 0;
    solar_os_json_doc_t *doc = NULL;
    const char *array = NULL;
    size_t array_len = 0;
    assert(solar_os_json_parse_object_except_array(catalog, len, "pages", &doc, &array,
                                                   &array_len) == ESP_OK);
    char schema[32];
    assert(solar_os_json_get_path_string(solar_os_json_root(doc), "schema", schema,
                                         sizeof(schema)) == ESP_OK);
    assert(strcmp(schema, "solaros.manual_catalog") == 0);
    solar_os_json_free(doc);

    size_t count = 0;
    assert(solar_os_json_array_for_each(array, array_len, NULL, NULL, &count) == ESP_OK);
    assert(count == page_total && count > 100);
    catalog_walk_t walk = {0};
    assert(solar_os_json_array_for_each(array, array_len, check_catalog_page, &walk,
                                        &count) == ESP_OK);
    assert(count == page_total && walk.pages == page_total);
    assert(live_bytes == 0);
    printf("manual catalog: %zu bytes, %zu pages, tree %zu bytes, page walk peak %zu bytes\n",
           len, count, tree_bytes, peak_bytes);
    assert(peak_bytes < 32U * 1024U);
    assert(peak_bytes * 20U < tree_bytes);
    free(catalog);
}

int main(int argc, char **argv)
{
    test_split_and_walk();
    test_empty_and_scalar_arrays();
    test_rejects_malformed_documents();
    assert(live_bytes == 0);
    if (argc > 1) {
        test_manual_catalog(argv[1]);
    }
    puts("json_stream_test passed");
    return 0;
}
