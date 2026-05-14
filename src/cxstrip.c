/*  cxstrip — strip a .cxe executable to a flat .cxbin
 *
 *  Usage:  cxstrip <input.cxe> [output.cxbin]
 *
 *  Reads every executable/data section from a .cxe file and writes them
 *  packed end-to-end starting at offset 0.  The resulting file can be
 *  loaded by cxvm (auto-detected or with --bin) at VM_TEXT_BASE.
 *
 *  Sections are laid out in memory-address order.  Gaps between sections
 *  are zero-filled so the flat image is coherent.  BSS (zero-fill) sections
 *  are expanded in-place.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "include/cxe.h"
#include "include/cxis.h"   /* VM_TEXT_BASE */

static int cmp_vaddr(const void *a, const void *b) {
    const CxeSection *sa = (const CxeSection *)a;
    const CxeSection *sb = (const CxeSection *)b;
    return (sa->vaddr < sb->vaddr) ? -1 : (sa->vaddr > sb->vaddr) ? 1 : 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,"usage: cxstrip <input.cxe> [output.cxbin]\n");
        return 1;
    }

    const char *inpath  = argv[1];
    char outbuf[512];
    if (argc >= 3) {
        snprintf(outbuf,sizeof(outbuf),"%s",argv[2]);
    } else {
        /* replace/append extension */
        snprintf(outbuf,sizeof(outbuf),"%s",inpath);
        char *dot = strrchr(outbuf,'.');
        if (dot) strcpy(dot,".cxbin");
        else      strcat(outbuf,".cxbin");
    }

    FILE *fin = fopen(inpath,"rb");
    if (!fin) { fprintf(stderr,"cxstrip: cannot open '%s': %s\n",inpath,strerror(errno)); return 1; }

    CxeHeader hdr;
    if (fread(&hdr,sizeof(hdr),1,fin) != 1 || hdr.magic != CXE_MAGIC) {
        fprintf(stderr,"cxstrip: '%s' is not a valid .cxe file\n",inpath);
        fclose(fin); return 1;
    }

    if (hdr.section_count == 0) {
        fprintf(stderr,"cxstrip: no sections in '%s'\n",inpath);
        fclose(fin); return 1;
    }

    /* read section headers */
    CxeSection *secs = calloc(hdr.section_count, sizeof(CxeSection));
    fread(secs, sizeof(CxeSection), hdr.section_count, fin);

    /* sort by vaddr */
    qsort(secs, hdr.section_count, sizeof(CxeSection), cmp_vaddr);

    /* determine image extent: from lowest vaddr to highest vaddr+mem_size */
    uint32_t img_base = secs[0].vaddr;
    uint32_t img_end  = 0;
    for (int i = 0; i < hdr.section_count; i++) {
        uint32_t end = secs[i].vaddr + secs[i].mem_size;
        if (end > img_end) img_end = end;
    }

    if (img_base < VM_TEXT_BASE) {
        fprintf(stderr,"cxstrip: warning: first section vaddr 0x%08X < VM_TEXT_BASE 0x%08X — "
                       "flat image will have leading zeros\n", img_base, VM_TEXT_BASE);
    }

    /* flat image relative to img_base */
    uint32_t img_size = img_end - img_base;
    uint8_t *img = calloc(1, img_size);
    if (!img) { fprintf(stderr,"cxstrip: out of memory\n"); free(secs); fclose(fin); return 1; }

    /* fill sections */
    for (int i = 0; i < hdr.section_count; i++) {
        CxeSection *s = &secs[i];
        uint32_t off = s->vaddr - img_base;
        if (s->flags & CXE_SEC_ZERO) {
            memset(img + off, 0, s->mem_size);
        } else {
            fseek(fin, s->offset, SEEK_SET);
            fread(img + off, 1, s->file_size, fin);
            if (s->mem_size > s->file_size)
                memset(img + off + s->file_size, 0, s->mem_size - s->file_size);
        }
    }

    fclose(fin);

    /* write flat image */
    FILE *fout = fopen(outbuf,"wb");
    if (!fout) { fprintf(stderr,"cxstrip: cannot write '%s': %s\n",outbuf,strerror(errno)); free(img); free(secs); return 1; }
    fwrite(img, 1, img_size, fout);
    fclose(fout);

    printf("cxstrip: %s → %s  (%u bytes, entry=0x%08X, base=0x%08X)\n",
           inpath, outbuf, img_size, hdr.entry_point, img_base);

    free(img); free(secs);
    return 0;
}
