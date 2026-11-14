#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "solar_os_json_scan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOLAR_OS_JSON_KEY_MAX 64

typedef struct solar_os_json_doc solar_os_json_doc_t;
typedef struct cJSON solar_os_json_value_t;

esp_err_t solar_os_json_init(void);
esp_err_t solar_os_json_parse(const char *source,
                              size_t source_len,
                              solar_os_json_doc_t **out_doc);
esp_err_t solar_os_json_parse_ex(const char *source,
                                 size_t source_len,
                                 solar_os_json_doc_t **out_doc,
                                 size_t *error_offset);
esp_err_t solar_os_json_parse_cstr(const char *source, solar_os_json_doc_t **out_doc);
void solar_os_json_free(solar_os_json_doc_t *doc);

/*
 * For a document whose array is too large to hold as one tree. Parses the
 * top-level object in source except the member array_key, which must be an
 * array: *array and *array_len receive that array's text inside source, and
 * the returned document holds the other members. Returns ESP_ERR_NOT_FOUND
 * when the object has no such member.
 */
esp_err_t solar_os_json_parse_object_except_array(const char *source,
                                                  size_t source_len,
                                                  const char *array_key,
                                                  solar_os_json_doc_t **out_doc,
                                                  const char **array,
                                                  size_t *array_len);

/*
 * Visits the elements of a JSON array text one at a time, so only one
 * element's tree exists at once. Setting *stop ends the walk early without
 * checking the rest of the array. Returns the first error from the parser or
 * from visit. With visit NULL, counts the elements without parsing them.
 * *count, when given, receives the number of elements visited.
 */
typedef esp_err_t (*solar_os_json_element_fn)(const solar_os_json_value_t *element,
                                              size_t index,
                                              bool *stop,
                                              void *user);
esp_err_t solar_os_json_array_for_each(const char *array,
                                       size_t array_len,
                                       solar_os_json_element_fn visit,
                                       void *user,
                                       size_t *count);

const solar_os_json_value_t *solar_os_json_root(const solar_os_json_doc_t *doc);
const solar_os_json_value_t *solar_os_json_object_get(const solar_os_json_value_t *object,
                                                      const char *key);
const solar_os_json_value_t *solar_os_json_path_get(const solar_os_json_value_t *value,
                                                    const char *path);
size_t solar_os_json_array_size(const solar_os_json_value_t *array);
const solar_os_json_value_t *solar_os_json_array_get(const solar_os_json_value_t *array,
                                                     size_t index);

bool solar_os_json_is_object(const solar_os_json_value_t *value);
bool solar_os_json_is_array(const solar_os_json_value_t *value);
bool solar_os_json_is_string(const solar_os_json_value_t *value);
bool solar_os_json_is_number(const solar_os_json_value_t *value);
bool solar_os_json_is_bool(const solar_os_json_value_t *value);
bool solar_os_json_is_null(const solar_os_json_value_t *value);

esp_err_t solar_os_json_get_string(const solar_os_json_value_t *value,
                                   char *out,
                                   size_t out_len);
/* The value's text while its document lives, or NULL if it is not a string. */
const char *solar_os_json_string_value(const solar_os_json_value_t *value);
esp_err_t solar_os_json_get_int64(const solar_os_json_value_t *value, int64_t *out);
esp_err_t solar_os_json_get_uint32(const solar_os_json_value_t *value, uint32_t *out);
esp_err_t solar_os_json_get_bool(const solar_os_json_value_t *value, bool *out);

esp_err_t solar_os_json_get_path_string(const solar_os_json_value_t *value,
                                        const char *path,
                                        char *out,
                                        size_t out_len);
esp_err_t solar_os_json_get_path_int64(const solar_os_json_value_t *value,
                                       const char *path,
                                       int64_t *out);
esp_err_t solar_os_json_get_path_uint32(const solar_os_json_value_t *value,
                                        const char *path,
                                        uint32_t *out);
esp_err_t solar_os_json_get_path_bool(const solar_os_json_value_t *value,
                                      const char *path,
                                      bool *out);

bool solar_os_json_array_contains_string(const solar_os_json_value_t *array,
                                         const char *text);
bool solar_os_json_path_array_contains_string(const solar_os_json_value_t *value,
                                              const char *path,
                                              const char *text);

#ifdef __cplusplus
}
#endif
