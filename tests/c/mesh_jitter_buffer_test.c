#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "shared/mesh_jitter_buffer.h"

static void push(mesh_jitter_buffer_t *buffer, char c, int64_t enqueued_us)
{
    uint8_t payload = (uint8_t)c;
    mesh_jitter_push(buffer, &payload, 1, 7, (uint8_t)c, 3, enqueued_us - 20,
                     enqueued_us);
}

static void expect_pop(mesh_jitter_buffer_t *buffer, int64_t now_us, char c)
{
    mesh_jitter_entry_t entry = {0};
    mesh_jitter_pop_result_t result = mesh_jitter_pop(buffer, now_us, &entry);
    assert(result.delivered && result.expired == 0);
    assert(entry.len == 1 && entry.data[0] == (uint8_t)c);
    assert(entry.src_id == 7 && entry.seq == (uint8_t)c && entry.audio_flags == 3);
    assert(entry.timestamp_us == entry.enqueued_us - 20);
}

int main(void)
{
    mesh_jitter_buffer_t buffer = {0};
    push(&buffer, 'A', 100);
    push(&buffer, 'B', 100);
    push(&buffer, 'C', 100);
    push(&buffer, 'D', 100);
    assert(buffer.count == 4);
    uint8_t e = 'E';
    assert(mesh_jitter_push(&buffer, &e, 1, 7, 'E', 3, 80, 100));
    for (char c = 'B'; c <= 'E'; c++) expect_pop(&buffer, 100, c);
    assert(buffer.count == 0 && buffer.read_idx == buffer.write_idx);
    push(&buffer, 'F', 101);
    expect_pop(&buffer, 101, 'F');

    for (int cycle = 0; cycle < 20; cycle++) {
        for (int i = 0; i < 7; i++) push(&buffer, (char)('A' + i), 200 + cycle);
        assert(buffer.count == 4);
        for (char c = 'D'; c <= 'G'; c++) expect_pop(&buffer, 200 + cycle, c);
        assert(buffer.count == 0 && buffer.read_idx == buffer.write_idx);
    }

    push(&buffer, 'X', 1000);
    push(&buffer, 'Y', 62000);
    mesh_jitter_entry_t entry = {0};
    mesh_jitter_pop_result_t result = mesh_jitter_pop(&buffer, 62000, &entry);
    assert(result.delivered && entry.data[0] == 'Y');
    assert(result.expired == 1 && result.expired_with_pending == 1);
    assert(result.expired_age_us_max == 61000 && buffer.count == 0);

    push(&buffer, 'A', 1000);
    push(&buffer, 'B', 2000);
    push(&buffer, 'C', 63000);
    result = mesh_jitter_pop(&buffer, 63000, &entry);
    assert(result.delivered && entry.data[0] == 'C');
    assert(result.expired == 2 && result.expired_with_pending == 2);
    assert(result.expired_age_us_max == 62000 && buffer.count == 0);

    push(&buffer, 'T', 1000);
    expect_pop(&buffer, 61000, 'T'); /* Exactly 60 ms is still fresh. */

    push(&buffer, 'X', 1000);
    push(&buffer, 'Y', 2000);
    result = mesh_jitter_pop(&buffer, 100000, &entry);
    assert(!result.delivered && result.expired == 2);
    assert(result.expired_with_pending == 0 && buffer.count == 0);
    assert(buffer.read_idx == buffer.write_idx);
    push(&buffer, 'Z', 100001);
    expect_pop(&buffer, 100001, 'Z');

    push(&buffer, 'P', 101000);
    push(&buffer, 'Q', 101000);
    assert(buffer.count == 2);
    mesh_jitter_reset(&buffer);
    assert(buffer.count == 0 && buffer.read_idx == 0 && buffer.write_idx == 0);
    push(&buffer, 'R', 101001);
    expect_pop(&buffer, 101001, 'R');
    puts("mesh_jitter_buffer_test: ok");
    return 0;
}
