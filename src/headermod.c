/*  headermod — inspect and patch .cxe / .cxbin / arbitrary file headers
 *
 *  Usage:
 *    headermod info   <file>                    show header fields
 *    headermod get    <file> <field>            print field value
 *    headermod set    <file> <field> <value>    patch field in-place
 *    headermod magic  <file> [new_magic_hex]    show/set magic bytes
 *    headermod entry  <file> [new_entry_hex]    show/set entry point (.cxe)
 *    headermod flags  <file> [new_flags_hex]    show/set flags (.cxe)
 *    headermod ver    <file> [new_ver_hex]      show/set version (.cxe)
 *    headermod hex    <file> <offset> <bytes>   patch raw bytes at offset
 *    headermod dump   <file> [n]                hex-dump first n bytes (default 64)
 *    headermod zero   <file> <offset> <len>     zero a byte range
 *    headermod copy   <file> <src_off> <dst_off> <len>  copy bytes within file
 *    headermod inject <file> <offset> <hexbytes>        overwrite bytes at offset
 *
 *  All numeric arguments accept decimal or 0x-prefixed hex.
 *
 *  Known .cxe fields: magic, version, flags, entry, sections
 *  Generic fields: u8@<off>, u16@<off>, u32@<off>, u64@<off>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <stddef.h>

#include "include/cxe.h"

/* ── helpers ──────────────────────────────────────────────────── */

static uint64_t parse_num(const char *s) {
    return (uint64_t)strtoull(s, NULL, 0);
}

static uint8_t *read_file(const char *path, long *out_sz) {
    FILE *f = fopen(path,"rb");
    if (!f) { fprintf(stderr,"headermod: cannot open '%s': %s\n",path,strerror(errno)); return NULL; }
    fseek(f,0,SEEK_END); *out_sz = ftell(f); rewind(f);
    uint8_t *buf = malloc(*out_sz);
    fread(buf,1,*out_sz,f); fclose(f); return buf;
}

static int write_file(const char *path, const uint8_t *buf, long sz) {
    FILE *f = fopen(path,"rb+");
    if (!f) { fprintf(stderr,"headermod: cannot open '%s' for writing: %s\n",path,strerror(errno)); return 0; }
    rewind(f);
    fwrite(buf,1,sz,f); fclose(f); return 1;
}

static void hexdump(const uint8_t *buf, long sz, long offset) {
    for (long i = 0; i < sz; i += 16) {
        printf("%08lX  ", offset + i);
        for (int j = 0; j < 16; j++) {
            if (i+j < sz) printf("%02X ",buf[i+j]); else printf("   ");
            if (j==7) printf(" ");
        }
        printf(" |");
        for (int j = 0; j < 16 && i+j < sz; j++)
            putchar(isprint(buf[i+j]) ? buf[i+j] : '.');
        printf("|\n");
    }
}

static void parse_hex_bytes(const char *s, uint8_t *out, int *out_len) {
    *out_len = 0;
    while (*s) {
        while (*s == ' ' || *s == ':') s++;
        if (!*s) break;
        char hi = *s++, lo = *s ? *s++ : '0';
        char tmp[3] = {hi, lo, 0};
        out[(*out_len)++] = (uint8_t)strtoul(tmp,NULL,16);
    }
}

/* ── CXE info printer ─────────────────────────────────────────── */

static void print_cxe_info(const uint8_t *buf, long sz) {
    if (sz < (long)sizeof(CxeHeader)) { printf("  (file too small for CxeHeader)\n"); return; }
    CxeHeader h; memcpy(&h, buf, sizeof(h));
    printf("  magic    : 0x%08X  (%c%c%c%c)\n", h.magic,
           (h.magic>>0)&0xFF,(h.magic>>8)&0xFF,(h.magic>>16)&0xFF,(h.magic>>24)&0xFF);
    printf("  version  : 0x%04X\n", h.version);
    printf("  flags    : 0x%04X\n", h.flags);
    printf("  entry    : 0x%08X\n", h.entry_point);
    printf("  sections : %u\n", h.section_count);
    for (int i = 0; i < h.section_count; i++) {
        long soff = sizeof(CxeHeader) + i*(long)sizeof(CxeSection);
        if (soff + (long)sizeof(CxeSection) > sz) { printf("  [section %d truncated]\n",i); break; }
        CxeSection s; memcpy(&s, buf+soff, sizeof(s));
        printf("  section %d: vaddr=0x%08X  file_off=0x%08X  fsize=%u  msize=%u  flags=0x%02X\n",
               i, s.vaddr, s.offset, s.file_size, s.mem_size, s.flags);
    }
}

/* ── generic field access ─────────────────────────────────────── */

/*  field syntax:
 *    magic, version, flags, entry, sections  → .cxe named fields
 *    u8@<off>, u16@<off>, u32@<off>, u64@<off>  → generic
 */

static long field_offset(const char *field, int *width) {
    if (strcmp(field,"magic")==0)    { *width=4; return offsetof(CxeHeader,magic); }
    if (strcmp(field,"version")==0)  { *width=2; return offsetof(CxeHeader,version); }
    if (strcmp(field,"flags")==0)    { *width=2; return offsetof(CxeHeader,flags); }
    if (strcmp(field,"entry")==0)    { *width=4; return offsetof(CxeHeader,entry_point); }
    if (strcmp(field,"sections")==0) { *width=2; return offsetof(CxeHeader,section_count); }
    /* u8@off / u16@off / u32@off / u64@off */
    if (strncmp(field,"u8@",3)==0)  { *width=1; return (long)parse_num(field+3); }
    if (strncmp(field,"u16@",4)==0) { *width=2; return (long)parse_num(field+4); }
    if (strncmp(field,"u32@",4)==0) { *width=4; return (long)parse_num(field+4); }
    if (strncmp(field,"u64@",4)==0) { *width=8; return (long)parse_num(field+4); }
    return -1;
}

static uint64_t read_le(const uint8_t *buf, int width) {
    uint64_t v = 0;
    for (int i = 0; i < width; i++) v |= ((uint64_t)buf[i]) << (8*i);
    return v;
}

static void write_le(uint8_t *buf, uint64_t v, int width) {
    for (int i = 0; i < width; i++) buf[i] = (uint8_t)((v >> (8*i)) & 0xFF);
}

/* ── main ─────────────────────────────────────────────────────── */

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr,
            "headermod — CXIS file header inspector / patcher\n\n"
            "usage:\n"
            "  headermod info   <file>\n"
            "  headermod get    <file> <field>\n"
            "  headermod set    <file> <field> <value>\n"
            "  headermod magic  <file> [new_hex]\n"
            "  headermod entry  <file> [new_hex]\n"
            "  headermod flags  <file> [new_hex]\n"
            "  headermod ver    <file> [new_hex]\n"
            "  headermod dump   <file> [n_bytes]\n"
            "  headermod hex    <file> <offset> <hexbytes>\n"
            "  headermod zero   <file> <offset> <len>\n"
            "  headermod copy   <file> <src> <dst> <len>\n"
            "  headermod inject <file> <offset> <hexbytes>\n\n"
            "fields: magic version flags entry sections  u8@N u16@N u32@N u64@N\n"
        );
        return 1;
    }

    const char *cmd  = argv[1];
    const char *path = argv[2];

    long sz;
    uint8_t *buf = read_file(path, &sz);
    if (!buf) return 1;

    int dirty = 0;

    /* ── info ── */
    if (strcmp(cmd,"info")==0) {
        printf("file : %s  (%ld bytes)\n", path, sz);
        /* detect type */
        if (sz >= 4) {
            uint32_t magic; memcpy(&magic, buf, 4);
            if (magic == CXE_MAGIC) {
                printf("type : .cxe executable\n");
                print_cxe_info(buf, sz);
            } else {
                printf("type : unknown / flat binary\n");
                printf("magic bytes: %02X %02X %02X %02X\n",
                    sz>0?buf[0]:0, sz>1?buf[1]:0, sz>2?buf[2]:0, sz>3?buf[3]:0);
            }
        }
        printf("---\nhex dump (first 64 bytes):\n");
        hexdump(buf, sz < 64 ? sz : 64, 0);
        goto done;
    }

    /* ── dump ── */
    if (strcmp(cmd,"dump")==0) {
        long n = (argc >= 4) ? (long)parse_num(argv[3]) : 64;
        if (n > sz) n = sz;
        hexdump(buf, n, 0);
        goto done;
    }

    /* ── get ── */
    if (strcmp(cmd,"get")==0) {
        if (argc < 4) { fprintf(stderr,"headermod get: need <field>\n"); goto fail; }
        int width; long off = field_offset(argv[3], &width);
        if (off < 0) { fprintf(stderr,"headermod: unknown field '%s'\n",argv[3]); goto fail; }
        if (off + width > sz) { fprintf(stderr,"headermod: field out of range\n"); goto fail; }
        uint64_t v = read_le(buf+off, width);
        printf("0x%0*llX  (%llu)\n", width*2, (unsigned long long)v, (unsigned long long)v);
        goto done;
    }

    /* ── set ── */
    if (strcmp(cmd,"set")==0 || strcmp(cmd,"magic")==0 || strcmp(cmd,"entry")==0
        || strcmp(cmd,"flags")==0 || strcmp(cmd,"ver")==0) {
        const char *field;
        const char *valstr;
        if (strcmp(cmd,"magic")==0) { field="magic"; valstr=argc>=4?argv[3]:NULL; }
        else if (strcmp(cmd,"entry")==0) { field="entry"; valstr=argc>=4?argv[3]:NULL; }
        else if (strcmp(cmd,"flags")==0) { field="flags"; valstr=argc>=4?argv[3]:NULL; }
        else if (strcmp(cmd,"ver")==0)   { field="version"; valstr=argc>=4?argv[3]:NULL; }
        else { field=argv[3]; valstr=argc>=5?argv[4]:NULL; }

        int width; long off = field_offset(field, &width);
        if (off < 0) { fprintf(stderr,"headermod: unknown field '%s'\n",field); goto fail; }
        if (off + width > sz) { fprintf(stderr,"headermod: field out of range\n"); goto fail; }

        if (!valstr) {
            /* read-only mode for single-arg magic/entry/flags/ver */
            uint64_t v = read_le(buf+off, width);
            printf("%s: 0x%0*llX  (%llu)\n", field, width*2,
                   (unsigned long long)v,(unsigned long long)v);
        } else {
            uint64_t newv = parse_num(valstr);
            uint64_t oldv = read_le(buf+off, width);
            write_le(buf+off, newv, width);
            printf("%s: 0x%0*llX → 0x%0*llX\n", field,
                   width*2,(unsigned long long)oldv,
                   width*2,(unsigned long long)newv);
            dirty = 1;
        }
        goto done;
    }

    /* ── hex / inject ── */
    if (strcmp(cmd,"hex")==0 || strcmp(cmd,"inject")==0) {
        if (argc < 5) { fprintf(stderr,"headermod %s: need <offset> <hexbytes>\n",cmd); goto fail; }
        long off = (long)parse_num(argv[3]);
        uint8_t patch[256]; int plen=0;
        parse_hex_bytes(argv[4], patch, &plen);
        if (off + plen > sz) { fprintf(stderr,"headermod: patch out of range\n"); goto fail; }
        printf("patching %d byte(s) at offset 0x%lX\n", plen, off);
        memcpy(buf+off, patch, plen);
        dirty = 1;
        goto done;
    }

    /* ── zero ── */
    if (strcmp(cmd,"zero")==0) {
        if (argc < 5) { fprintf(stderr,"headermod zero: need <offset> <len>\n"); goto fail; }
        long off = (long)parse_num(argv[3]);
        long len = (long)parse_num(argv[4]);
        if (off + len > sz) { fprintf(stderr,"headermod: range out of bounds\n"); goto fail; }
        memset(buf+off, 0, len);
        printf("zeroed %ld byte(s) at 0x%lX\n", len, off);
        dirty = 1;
        goto done;
    }

    /* ── copy ── */
    if (strcmp(cmd,"copy")==0) {
        if (argc < 6) { fprintf(stderr,"headermod copy: need <src> <dst> <len>\n"); goto fail; }
        long src = (long)parse_num(argv[3]);
        long dst = (long)parse_num(argv[4]);
        long len = (long)parse_num(argv[5]);
        if (src+len > sz || dst+len > sz) { fprintf(stderr,"headermod: copy out of bounds\n"); goto fail; }
        memmove(buf+dst, buf+src, len);
        printf("copied %ld byte(s) from 0x%lX to 0x%lX\n", len, src, dst);
        dirty = 1;
        goto done;
    }

    /* ── sections subcommand ── */
    if (strcmp(cmd,"sections")==0) {
        print_cxe_info(buf, sz);
        goto done;
    }

    fprintf(stderr,"headermod: unknown command '%s'\n", cmd);
    goto fail;

done:
    if (dirty) {
        if (!write_file(path, buf, sz)) goto fail;
        printf("saved: %s\n", path);
    }
    free(buf);
    return 0;
fail:
    free(buf);
    return 1;
}
