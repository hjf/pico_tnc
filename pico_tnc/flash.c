#if PICO_TNC_PARENT_INTEGRATION
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define PICO_MAGIC 0x4f434950

extern char __flash_binary_end;

static uint8_t *flash_addr(void)
{
    return (uint8_t *)(((uint32_t)&__flash_binary_end + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1));
}

static uint8_t *flash_find_id(void)
{
    uint8_t *addr = flash_addr();
    uint8_t *id = NULL;

    for (int page = 0; page < FLASH_SECTOR_SIZE; page += FLASH_PAGE_SIZE) {
        if (*(uint32_t *)&addr[page] == PICO_MAGIC)  {
            id = &addr[page];
        } else {
            break;
        }
    }
    return id; // return last one
}

static void flash_erase(void)
{
    uint32_t flash_offset = (uint32_t)flash_addr() - XIP_BASE;
    uint32_t int_save = save_and_disable_interrupts();

    busy_wait_us_32(8334); // wait 10 bit period

    flash_range_erase(flash_offset, FLASH_SECTOR_SIZE);
    restore_interrupts(int_save);
}

static void flash_program(uint8_t *flash, uint8_t *data)
{
    if (flash < flash_addr()) return;
    if (flash >= (uint8_t *)0x10200000) return; // > 2MB

    uint32_t flash_offset = (uint32_t)flash - XIP_BASE;
    uint32_t int_save = save_and_disable_interrupts();

    busy_wait_us_32(8334); // wait 10 bit period

    flash_range_program(flash_offset, data, FLASH_PAGE_SIZE);
    restore_interrupts(int_save);
}

bool flash_read(void *data, int len)
{
    if (len > FLASH_PAGE_SIZE - sizeof(uint32_t)) return false;

    uint8_t *src = flash_find_id();

    if (src == NULL) return false;

    memcpy(data, src + sizeof(uint32_t), len);
    return true;
}

bool flash_write(void *data, int len)
{
    if (len > FLASH_PAGE_SIZE - sizeof(uint32_t)) return false;

    uint8_t *mem = calloc(1, FLASH_PAGE_SIZE);

    if (mem == NULL) return false;

    uint32_t id = PICO_MAGIC;

    memcpy(mem, &id, sizeof(id));
    memcpy(mem + sizeof(id), data, len);

    uint8_t *dst = flash_find_id();

    if (!dst) { // flash not initialized

        flash_erase();
        dst = flash_addr();

    } else {

        dst += FLASH_PAGE_SIZE;

        if (dst - flash_addr() >= FLASH_SECTOR_SIZE) { // no writable area

            flash_erase();
            dst = flash_addr();

        }
    }

    flash_program(dst, mem);

    bool matched = !memcmp(dst, mem, FLASH_PAGE_SIZE);

    free(mem);

    return matched;
}

#else
// Configuration is a checksummed record in alternating end-of-flash sectors.
// Never erase the last valid copy. Legacy magic-only records are not trusted.
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "flash.h"

#define CONFIG_MAGIC 0x33434e54u
#define CONFIG_BASE (PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE)
#define HEADER_SIZE 16
extern char __flash_binary_end;

typedef struct {
    uint32_t magic, sequence, length, crc;
    uint8_t data[FLASH_PAGE_SIZE - HEADER_SIZE];
} config_record_t;
_Static_assert(sizeof(config_record_t) == FLASH_PAGE_SIZE, "flash page layout");

static uint32_t checksum(const config_record_t *r)
{
    uint32_t crc = ~0u;
    const uint8_t *p = (const uint8_t *)r;
    for (unsigned i = 0; i < sizeof(*r); ++i) {
        if (i >= 12 && i < 16) continue;
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

static const config_record_t *record(int slot)
{
    return (const config_record_t *)(XIP_BASE + CONFIG_BASE + slot * FLASH_SECTOR_SIZE);
}
static bool valid(const config_record_t *r)
{
    return r->magic == CONFIG_MAGIC && r->length <= sizeof(r->data) && r->crc == checksum(r);
}
static int newest(void)
{
    bool a = valid(record(0)), b = valid(record(1));
    if (!a) return b ? 1 : -1;
    if (!b) return 0;
    return (int32_t)(record(1)->sequence - record(0)->sequence) > 0 ? 1 : 0;
}
bool flash_read(void *data, int len)
{
    if (!data || len < 0 || len > (int)sizeof(record(0)->data)) return false;
    if ((uintptr_t)&__flash_binary_end > XIP_BASE + CONFIG_BASE) return false;
    int slot = newest();
    if (slot < 0 || (uint32_t)len > record(slot)->length) return false;
    memcpy(data, record(slot)->data, len);
    return true;
}
bool flash_write(void *data, int len)
{
    if (!data || len < 0 || len > (int)sizeof(record(0)->data)) return false;
    if ((uintptr_t)&__flash_binary_end > XIP_BASE + CONFIG_BASE) return false;
    int old = newest(), slot = old == 0 ? 1 : 0;
    config_record_t next = { .magic = CONFIG_MAGIC, .length = len,
                            .sequence = old < 0 ? 1 : record(old)->sequence + 1 };
    memcpy(next.data, data, len);
    next.crc = checksum(&next);
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(CONFIG_BASE + slot * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_BASE + slot * FLASH_SECTOR_SIZE, (uint8_t *)&next, sizeof(next));
    restore_interrupts(irq);
    return valid(record(slot)) && !memcmp(record(slot), &next, sizeof(next));
}

#endif
