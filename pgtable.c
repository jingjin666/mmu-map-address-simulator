#include <stdio.h>

#define __AC(X,Y)	(X##Y)
#define _AC(X,Y)	__AC(X,Y)
#define _AT(T,X)	((T)(X))

#define _UL(x)		(_AC(x, UL))
#define _ULL(x)		(_AC(x, ULL))
  
#define UL(x)		(_UL(x))
#define ULL(x)		(_ULL(x))
  
#define BIT(nr)		(UL(1) << (nr))

#define CONFIG_ARM64_PAGE_SHIFT 12

#define PAGE_SHIFT		CONFIG_ARM64_PAGE_SHIFT
#define PAGE_SIZE		(_AC(1, UL) << PAGE_SHIFT)
#define PAGE_MASK		(~(PAGE_SIZE-1))

#define CONFIG_PGTABLE_LEVELS 4
#define VA_BITS 48
/*
 * Number of page-table levels required to address 'va_bits' wide
 * address, without section mapping. We resolve the top (va_bits - PAGE_SHIFT)
 * bits with (PAGE_SHIFT - 3) bits at each page table level. Hence:
 *
 *  levels = DIV_ROUND_UP((va_bits - PAGE_SHIFT), (PAGE_SHIFT - 3))
 *
 * where DIV_ROUND_UP(n, d) => (((n) + (d) - 1) / (d))
 *
 * We cannot include linux/kernel.h which defines DIV_ROUND_UP here
 * due to build issues. So we open code DIV_ROUND_UP here:
 *
 *	((((va_bits) - PAGE_SHIFT) + (PAGE_SHIFT - 3) - 1) / (PAGE_SHIFT - 3))
 *
 * which gets simplified as :
 */
#define ARM64_HW_PGTABLE_LEVELS(va_bits) (((va_bits) - 4) / (PAGE_SHIFT - 3))

/*
 * Size mapped by an entry at level n ( 0 <= n <= 3)
 * We map (PAGE_SHIFT - 3) at all translation levels and PAGE_SHIFT bits
 * in the final page. The maximum number of translation levels supported by
 * the architecture is 4. Hence, starting at level n, we have further
 * ((4 - n) - 1) levels of translation excluding the offset within the page.
 * So, the total number of bits mapped by an entry at level n is :
 *
 *  ((4 - n) - 1) * (PAGE_SHIFT - 3) + PAGE_SHIFT
 *
 * Rearranging it a bit we get :
 *   (4 - n) * (PAGE_SHIFT - 3) + 3
 */
#define ARM64_HW_PGTABLE_LEVEL_SHIFT(n)	((PAGE_SHIFT - 3) * (4 - (n)) + 3)

#define PTRS_PER_PTE		(1 << (PAGE_SHIFT - 3))

/*
 * PMD_SHIFT determines the size a level 2 page table entry can map.
 */
#if CONFIG_PGTABLE_LEVELS > 2
#define PMD_SHIFT		ARM64_HW_PGTABLE_LEVEL_SHIFT(2)
#define PMD_SIZE		(_AC(1, UL) << PMD_SHIFT)
#define PMD_MASK		(~(PMD_SIZE-1))
#define PTRS_PER_PMD		PTRS_PER_PTE
#endif

/*
 * PUD_SHIFT determines the size a level 1 page table entry can map.
 */
#if CONFIG_PGTABLE_LEVELS > 3
#define PUD_SHIFT		ARM64_HW_PGTABLE_LEVEL_SHIFT(1)
#define PUD_SIZE		(_AC(1, UL) << PUD_SHIFT)
#define PUD_MASK		(~(PUD_SIZE-1))
#define PTRS_PER_PUD		PTRS_PER_PTE
#endif

/*
 * PGDIR_SHIFT determines the size a top-level page table entry can map
 * (depending on the configuration, this level can be 0, 1 or 2).
 */
#define PGDIR_SHIFT		ARM64_HW_PGTABLE_LEVEL_SHIFT(4 - CONFIG_PGTABLE_LEVELS)
#define PGDIR_SIZE		(_AC(1, UL) << PGDIR_SHIFT)
#define PGDIR_MASK		(~(PGDIR_SIZE-1))
#define PTRS_PER_PGD		(1 << (VA_BITS - PGDIR_SHIFT))

/*
 * Section address mask and size definitions.
 */
#define SECTION_SHIFT		PMD_SHIFT
#define SECTION_SIZE		(_AC(1, UL) << SECTION_SHIFT)
#define SECTION_MASK		(~(SECTION_SIZE-1))

/*
 * Contiguous page definitions.
 */
#ifdef CONFIG_ARM64_64K_PAGES
#define CONT_PTE_SHIFT		(5 + PAGE_SHIFT)
#define CONT_PMD_SHIFT		(5 + PMD_SHIFT)
#elif defined(CONFIG_ARM64_16K_PAGES)
#define CONT_PTE_SHIFT		(7 + PAGE_SHIFT)
#define CONT_PMD_SHIFT		(5 + PMD_SHIFT)
#else
#define CONT_PTE_SHIFT		(4 + PAGE_SHIFT)
#define CONT_PMD_SHIFT		(4 + PMD_SHIFT)
#endif

#define CONT_PTES		(1 << (CONT_PTE_SHIFT - PAGE_SHIFT))
#define CONT_PTE_SIZE		(CONT_PTES * PAGE_SIZE)
#define CONT_PTE_MASK		(~(CONT_PTE_SIZE - 1))
#define CONT_PMDS		(1 << (CONT_PMD_SHIFT - PMD_SHIFT))
#define CONT_PMD_SIZE		(CONT_PMDS * PMD_SIZE)
#define CONT_PMD_MASK		(~(CONT_PMD_SIZE - 1))
/* the numerical offset of the PTE within a range of CONT_PTES */
#define CONT_RANGE_OFFSET(addr) (((addr)>>PAGE_SHIFT)&(CONT_PTES-1))

/*
 * Hardware page table definitions.
 *
 * Level 1 descriptor (PUD).
 */
#define PUD_TYPE_TABLE		(_AT(pudval_t, 3) << 0)
#define PUD_TABLE_BIT		(_AT(pudval_t, 1) << 1)
#define PUD_TYPE_MASK		(_AT(pudval_t, 3) << 0)
#define PUD_TYPE_SECT		(_AT(pudval_t, 1) << 0)
#define PUD_SECT_RDONLY		(_AT(pudval_t, 1) << 7)		/* AP[2] */

/*
 * Level 2 descriptor (PMD).
 */
#define PMD_TYPE_MASK		(_AT(pmdval_t, 3) << 0)
#define PMD_TYPE_TABLE		(_AT(pmdval_t, 3) << 0)
#define PMD_TYPE_SECT		(_AT(pmdval_t, 1) << 0)
#define PMD_TABLE_BIT		(_AT(pmdval_t, 1) << 1)

/*
 * Section
 */
#define PMD_SECT_VALID		(_AT(pmdval_t, 1) << 0)
#define PMD_SECT_USER		(_AT(pmdval_t, 1) << 6)		/* AP[1] */
#define PMD_SECT_RDONLY		(_AT(pmdval_t, 1) << 7)		/* AP[2] */
#define PMD_SECT_S		(_AT(pmdval_t, 3) << 8)
#define PMD_SECT_AF		(_AT(pmdval_t, 1) << 10)
#define PMD_SECT_NG		(_AT(pmdval_t, 1) << 11)
#define PMD_SECT_CONT		(_AT(pmdval_t, 1) << 52)
#define PMD_SECT_PXN		(_AT(pmdval_t, 1) << 53)
#define PMD_SECT_UXN		(_AT(pmdval_t, 1) << 54)

/*
 * AttrIndx[2:0] encoding (mapping attributes defined in the MAIR* registers).
 */
#define PMD_ATTRINDX(t)		(_AT(pmdval_t, (t)) << 2)
#define PMD_ATTRINDX_MASK	(_AT(pmdval_t, 7) << 2)

/*
 * Level 3 descriptor (PTE).
 */
#define PTE_VALID		(_AT(pteval_t, 1) << 0)
#define PTE_TYPE_MASK	(_AT(pteval_t, 3) << 0)
#define PTE_TYPE_PAGE	(_AT(pteval_t, 3) << 0)
#define PTE_TABLE_BIT	(_AT(pteval_t, 1) << 1)
#define PTE_USER		(_AT(pteval_t, 1) << 6)		/* AP[1] */
#define PTE_RDONLY		(_AT(pteval_t, 1) << 7)		/* AP[2] */
#define PTE_SHARED		(_AT(pteval_t, 3) << 8)		/* SH[1:0], inner shareable */
#define PTE_AF			(_AT(pteval_t, 1) << 10)	/* Access Flag */
#define PTE_NG			(_AT(pteval_t, 1) << 11)	/* nG */
#define PTE_GP			(_AT(pteval_t, 1) << 50)	/* BTI guarded */
#define PTE_DBM			(_AT(pteval_t, 1) << 51)	/* Dirty Bit Management */
#define PTE_CONT		(_AT(pteval_t, 1) << 52)	/* Contiguous range */
#define PTE_PXN			(_AT(pteval_t, 1) << 53)	/* Privileged XN */
#define PTE_UXN			(_AT(pteval_t, 1) << 54)	/* User XN */
#define PTE_HYP_XN		(_AT(pteval_t, 1) << 54)	/* HYP XN */

#define PTE_ADDR_LOW (((1UL << (48 - PAGE_SHIFT)) - 1) << PAGE_SHIFT)

#ifdef MALLOC_ALIGN_4K
#define PTE_ADDR_MASK PTE_ADDR_LOW
#else
#define PTE_ADDR_MASK (~lowbitsmask(2))
#endif

/*
 * AttrIndx[2:0] encoding (mapping attributes defined in the MAIR* registers).
 */
#define PTE_ATTRINDX(t)		(_AT(pteval_t, (t)) << 2)
#define PTE_ATTRINDX_MASK	(_AT(pteval_t, 7) << 2)

/*
 * 2nd stage PTE definitions
 */
#define PTE_S2_RDONLY		(_AT(pteval_t, 1) << 6)   /* HAP[2:1] */
#define PTE_S2_RDWR		(_AT(pteval_t, 3) << 6)   /* HAP[2:1] */
#define PTE_S2_XN		(_AT(pteval_t, 2) << 53)  /* XN[1:0] */
#define PTE_S2_SW_RESVD		(_AT(pteval_t, 15) << 55) /* Reserved for SW */

#define PMD_S2_RDONLY		(_AT(pmdval_t, 1) << 6)   /* HAP[2:1] */
#define PMD_S2_RDWR		(_AT(pmdval_t, 3) << 6)   /* HAP[2:1] */
#define PMD_S2_XN		(_AT(pmdval_t, 2) << 53)  /* XN[1:0] */
#define PMD_S2_SW_RESVD		(_AT(pmdval_t, 15) << 55) /* Reserved for SW */

#define PUD_S2_RDONLY		(_AT(pudval_t, 1) << 6)   /* HAP[2:1] */
#define PUD_S2_RDWR		(_AT(pudval_t, 3) << 6)   /* HAP[2:1] */
#define PUD_S2_XN		(_AT(pudval_t, 2) << 53)  /* XN[1:0] */

/*
 * Memory Attribute override for Stage-2 (MemAttr[3:0])
 */
#define PTE_S2_MEMATTR(t)	(_AT(pteval_t, (t)) << 2)

/*
 * EL2/HYP PTE/PMD definitions
 */
#define PMD_HYP			PMD_SECT_USER
#define PTE_HYP			PTE_USER

/*
 * Highest possible physical address supported.
 */
#define PHYS_MASK_SHIFT		(CONFIG_ARM64_PA_BITS)
#define PHYS_MASK		((UL(1) << PHYS_MASK_SHIFT) - 1)

#define TTBR_CNP_BIT		(UL(1) << 0)

/*
 * TCR flags.
 */
#define TCR_T0SZ_OFFSET		0
#define TCR_T1SZ_OFFSET		16
#define TCR_T0SZ(x)		((UL(64) - (x)) << TCR_T0SZ_OFFSET)
#define TCR_T1SZ(x)		((UL(64) - (x)) << TCR_T1SZ_OFFSET)
#define TCR_TxSZ(x)		(TCR_T0SZ(x) | TCR_T1SZ(x))
#define TCR_TxSZ_WIDTH		6
#define TCR_T0SZ_MASK		(((UL(1) << TCR_TxSZ_WIDTH) - 1) << TCR_T0SZ_OFFSET)
#define TCR_T1SZ_MASK		(((UL(1) << TCR_TxSZ_WIDTH) - 1) << TCR_T1SZ_OFFSET)

#define TCR_EPD0_SHIFT		7
#define TCR_EPD0_MASK		(UL(1) << TCR_EPD0_SHIFT)
#define TCR_IRGN0_SHIFT		8
#define TCR_IRGN0_MASK		(UL(3) << TCR_IRGN0_SHIFT)
#define TCR_IRGN0_NC		(UL(0) << TCR_IRGN0_SHIFT)
#define TCR_IRGN0_WBWA		(UL(1) << TCR_IRGN0_SHIFT)
#define TCR_IRGN0_WT		(UL(2) << TCR_IRGN0_SHIFT)
#define TCR_IRGN0_WBnWA		(UL(3) << TCR_IRGN0_SHIFT)

#define TCR_EPD1_SHIFT		23
#define TCR_EPD1_MASK		(UL(1) << TCR_EPD1_SHIFT)
#define TCR_IRGN1_SHIFT		24
#define TCR_IRGN1_MASK		(UL(3) << TCR_IRGN1_SHIFT)
#define TCR_IRGN1_NC		(UL(0) << TCR_IRGN1_SHIFT)
#define TCR_IRGN1_WBWA		(UL(1) << TCR_IRGN1_SHIFT)
#define TCR_IRGN1_WT		(UL(2) << TCR_IRGN1_SHIFT)
#define TCR_IRGN1_WBnWA		(UL(3) << TCR_IRGN1_SHIFT)

#define TCR_IRGN_NC		(TCR_IRGN0_NC | TCR_IRGN1_NC)
#define TCR_IRGN_WBWA		(TCR_IRGN0_WBWA | TCR_IRGN1_WBWA)
#define TCR_IRGN_WT		(TCR_IRGN0_WT | TCR_IRGN1_WT)
#define TCR_IRGN_WBnWA		(TCR_IRGN0_WBnWA | TCR_IRGN1_WBnWA)
#define TCR_IRGN_MASK		(TCR_IRGN0_MASK | TCR_IRGN1_MASK)


#define TCR_ORGN0_SHIFT		10
#define TCR_ORGN0_MASK		(UL(3) << TCR_ORGN0_SHIFT)
#define TCR_ORGN0_NC		(UL(0) << TCR_ORGN0_SHIFT)
#define TCR_ORGN0_WBWA		(UL(1) << TCR_ORGN0_SHIFT)
#define TCR_ORGN0_WT		(UL(2) << TCR_ORGN0_SHIFT)
#define TCR_ORGN0_WBnWA		(UL(3) << TCR_ORGN0_SHIFT)

#define TCR_ORGN1_SHIFT		26
#define TCR_ORGN1_MASK		(UL(3) << TCR_ORGN1_SHIFT)
#define TCR_ORGN1_NC		(UL(0) << TCR_ORGN1_SHIFT)
#define TCR_ORGN1_WBWA		(UL(1) << TCR_ORGN1_SHIFT)
#define TCR_ORGN1_WT		(UL(2) << TCR_ORGN1_SHIFT)
#define TCR_ORGN1_WBnWA		(UL(3) << TCR_ORGN1_SHIFT)

#define TCR_ORGN_NC		(TCR_ORGN0_NC | TCR_ORGN1_NC)
#define TCR_ORGN_WBWA		(TCR_ORGN0_WBWA | TCR_ORGN1_WBWA)
#define TCR_ORGN_WT		(TCR_ORGN0_WT | TCR_ORGN1_WT)
#define TCR_ORGN_WBnWA		(TCR_ORGN0_WBnWA | TCR_ORGN1_WBnWA)
#define TCR_ORGN_MASK		(TCR_ORGN0_MASK | TCR_ORGN1_MASK)

#define TCR_SH0_SHIFT		12
#define TCR_SH0_MASK		(UL(3) << TCR_SH0_SHIFT)
#define TCR_SH0_INNER		(UL(3) << TCR_SH0_SHIFT)

#define TCR_SH1_SHIFT		28
#define TCR_SH1_MASK		(UL(3) << TCR_SH1_SHIFT)
#define TCR_SH1_INNER		(UL(3) << TCR_SH1_SHIFT)
#define TCR_SHARED		(TCR_SH0_INNER | TCR_SH1_INNER)

#define TCR_TG0_SHIFT		14
#define TCR_TG0_MASK		(UL(3) << TCR_TG0_SHIFT)
#define TCR_TG0_4K		(UL(0) << TCR_TG0_SHIFT)
#define TCR_TG0_64K		(UL(1) << TCR_TG0_SHIFT)
#define TCR_TG0_16K		(UL(2) << TCR_TG0_SHIFT)

#define TCR_TG1_SHIFT		30
#define TCR_TG1_MASK		(UL(3) << TCR_TG1_SHIFT)
#define TCR_TG1_16K		(UL(1) << TCR_TG1_SHIFT)
#define TCR_TG1_4K		(UL(2) << TCR_TG1_SHIFT)
#define TCR_TG1_64K		(UL(3) << TCR_TG1_SHIFT)

#define TCR_IPS_SHIFT		32
#define TCR_IPS_MASK		(UL(7) << TCR_IPS_SHIFT)
#define TCR_A1			(UL(1) << 22)
#define TCR_ASID16		(UL(1) << 36)
#define TCR_TBI0		(UL(1) << 37)
#define TCR_TBI1		(UL(1) << 38)
#define TCR_HA			(UL(1) << 39)
#define TCR_HD			(UL(1) << 40)
#define TCR_NFD0		(UL(1) << 53)
#define TCR_NFD1		(UL(1) << 54)
#define TCR_E0PD0		(UL(1) << 55)
#define TCR_E0PD1		(UL(1) << 56)

#define bitmask(x) (UL(1) << (x))
#define lowbitsmask(x) (bitmask(x) - UL(1))

#define align_to(size, align) ((((size) + (align) - 1) / (align))*(align))

typedef unsigned long u64;

typedef u64 pteval_t;
typedef u64 pmdval_t;
typedef u64 pudval_t;
typedef u64 pgdval_t;

/*
 * These are used to make use of C type-checking..
 */
typedef struct { pteval_t pte; } pte_t;
#define pte_val(x)	((x).pte)
#define __pte(x)	((pte_t) { (x) } )

typedef struct { pmdval_t pmd; } pmd_t;
#define pmd_val(x)	((x).pmd)
#define __pmd(x)	((pmd_t) { (x) } )

typedef struct { pudval_t pud; } pud_t;
#define pud_val(x)	((x).pud)
#define __pud(x)	((pud_t) { (x) } )

typedef struct { pgdval_t pgd; } pgd_t;
#define pgd_val(x)	((x).pgd)
#define __pgd(x)	((pgd_t) { (x) } )

typedef struct { pteval_t pgprot; } pgprot_t;
#define pgprot_val(x)	((x).pgprot)
#define __pgprot(x)	((pgprot_t) { (x) } )

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#define kprintf printf

u64 pg_alloc(u64 size)
{
    return (u64)malloc(size);
}

int pte_map(pmd_t *pmd_p, u64 vaddr, u64 end, u64 paddr, u64 attr)
{
    int ret = 0;
    pmd_t *pmd = pmd_p;
    pte_t *pte;
    u64 next, idx;

    if (!pmd_val(*pmd)) {
        // PMD页表中不存在PTE,需要重新分配PTE
        u64 pte_paddr = pg_alloc(PAGE_SIZE);
        //kprintf("pte_paddr = 0x%lx\n", pte_paddr);
        // 把PTE添加到PMD页表描符中去
        *pmd = __pmd(pte_paddr | PMD_TYPE_TABLE);
        pte = (pte_t *)pte_paddr;
    } else {
        // 存在PTE
        pte = (pte_t *)(pmd_val(*pmd) & PTE_ADDR_MASK);
    }

    // 获取PTE索引
    idx = (vaddr >> PAGE_SHIFT) & (PTRS_PER_PTE - 1);
    // 更新PTE起始地址
    pte += idx;

    for (next = vaddr; next != end; pte++) {
        next = (next + PAGE_SIZE) & PAGE_MASK;
        next = next < end ? next : end;
        *pte = __pte(((paddr >> PAGE_SHIFT) << PAGE_SHIFT) | PTE_TYPE_PAGE | attr);
        paddr += next - vaddr;
        vaddr = next;
    }

    return ret;
}


int pmd_map(pud_t *pud_p, u64 vaddr, u64 end, u64 paddr, u64 attr)
{
    int ret = 0;
    pud_t *pud = pud_p;
    pmd_t *pmd;
    u64 next, idx;

    if (!pud_val(*pud)) {
        // PUD页表中不存在PMU,需要重新分配PMD
        u64 pmd_paddr = pg_alloc(PAGE_SIZE);
        //kprintf("pmd_paddr = 0x%lx\n", pmd_paddr);
        // 把PMD添加到PUD页表描符中去
        *pud = __pud(pmd_paddr | PMD_TYPE_TABLE);
        //kprintf("--%p = 0x%lx\n", pud,  pud_val(*pud));
        pmd = (pmd_t *)pmd_paddr;
    } else {
        // 存在PMD
        pmd = (pmd_t *)(pud_val(*pud) & PTE_ADDR_MASK);
    }

    // 获取PMD索引
    idx = (vaddr >> PMD_SHIFT) & (PTRS_PER_PMD - 1);
    // 更新PMD起始地址
    pmd += idx;

    for (next = vaddr; next != end; pmd++) {
        next = (next + PMD_SIZE) & PMD_MASK;
        next = next < end ? next : end;

        if ( (vaddr | next | paddr) & ~SECTION_MASK) {
            // 使用页式映射
            pte_map(pmd, vaddr, next, paddr, attr);
        } else {
             // 如果地址全部段对齐,使用段氏映射
            *pmd = __pmd(((paddr >> PMD_SHIFT) << PMD_SHIFT) | PMD_TYPE_SECT | (attr & ~PTE_TABLE_BIT));
        }
        paddr += next - vaddr;
        vaddr = next;
    }

    return ret;

}

int pud_map(pgd_t *pgd_p, u64 vaddr, u64 end, u64 paddr, u64 attr)
{
    int ret = 0;
    pgd_t *pgd = pgd_p;
    pud_t *pud;
    u64 next, idx;

    if (!pgd_val(*pgd)) {
        // PGD页表中不存在PUD,需要重新分配PUD
        u64 pud_paddr = pg_alloc(PAGE_SIZE);
        //kprintf("pud_paddr = 0x%lx\n", pud_paddr);
        // 把PUD添加到PGD页表描符中去
        *pgd = __pgd(pud_paddr | PUD_TYPE_TABLE);
        //kprintf("--%p = 0x%lx\n", pgd,  pgd_val(*pgd));
        pud = (pud_t *)pud_paddr;
    } else {
        // 存在PUD
        pud = (pud_t *)(pgd_val(*pgd) & PTE_ADDR_MASK);
    }

    // 获取PUD索引
    idx = (vaddr >> PUD_SHIFT) & (PTRS_PER_PUD - 1);
    // 更新PUD起始地址
    pud += idx;

    for (next = vaddr; next != end; pud++) {
        next = (next + PUD_SIZE) & PUD_MASK;
        next = next < end ? next : end;
        pmd_map(pud, vaddr, next, paddr, attr);
        paddr += next - vaddr;
        vaddr = next;
    }

    return ret;
}

int pg_map(pgd_t *pgd_p, u64 vaddr, u64 paddr, u64 size, u64 attr)
{
    int ret = 0;
    u64 next, end, idx;
    pgd_t *pgd = pgd_p;

    assert(pgd_p);
    assert(!(vaddr & lowbitsmask(PAGE_SHIFT)));
    assert(!(paddr & lowbitsmask(PAGE_SHIFT)));
    assert(size);

    // 获取PGD索引
    idx = (vaddr >> PGDIR_SHIFT) & (PTRS_PER_PGD - 1);

    // 更新PGD起始地址
    pgd += idx;

    // 对齐结束地址
    end = align_to(vaddr + size, PAGE_SIZE);

    kprintf("idx = %ld, pgd = %p, vaddr = 0x%lx, paddr = 0x%lx, end = 0x%lx\n", idx, pgd, vaddr, paddr, end);

    for (next = vaddr; next != end; pgd++) {
        next = (next + PGDIR_SIZE) & PGDIR_MASK;
        next = next < end ? next : end;
        pud_map(pgd, vaddr, next, paddr, attr);
        paddr += next - vaddr;
        vaddr = next;
    }

    return ret;
}

void dump_pgtable_brief(pgd_t *pgdp)
{
    pgd_t *pgd = pgdp;
    pud_t *pud = NULL;
    pmd_t *pmd = NULL;
    pte_t *pte = NULL;
    u64  i, j, k, l;

    kprintf("{\n");

    int pgd_flag = 1, pud_flag = 1, pmd_flag = 1, pte_flag = 1;

    for (i = 0; i < PTRS_PER_PGD; i++, pgd++)
    {
        if (pgd_val(*pgd)) {
            if (pgd_flag) {
                pgd_flag = 0;
                kprintf("   \"pgd\": [\n");
            }
            kprintf("       {\n");
            kprintf("           \"idx\": \"%ld\",\n", i);
            kprintf("           \"addr\": \"%p\",\n", pgd);
            kprintf("           \"value\": \"0x%lx\",\n", pgd_val(*pgd));
            //kprintf("pgd %p [%ld] = 0x%lx\n", pgd, i, pgd_val(*pgd));
            pud = (pud_t *)(pgd_val(*pgd) & PTE_ADDR_MASK );
            for (j = 0; j < PTRS_PER_PUD; j++, pud++) 
            {
                if (pud_val(*pud)) {
                    if (pud_flag) {
                        pud_flag = 0;
                        kprintf("           \"pud\": [\n");
                    }
                    kprintf("               {\n");
                    kprintf("                   \"idx\": \"%ld\",\n", j);
                    kprintf("                   \"addr\": \"%p\",\n", pud);
                    kprintf("                   \"value\": \"0x%lx\",\n", pud_val(*pud));
                    //kprintf("---->pud %p [%ld] = 0x%lx\n", pud, j, pud_val(*pud));
                    pmd = (pmd_t *)(pud_val(*pud)  & PTE_ADDR_MASK);
                    for (k = 0; k < PTRS_PER_PMD; k++, pmd++) 
                    {
                        if (pmd_val(*pmd)) {
                            if (pmd_flag) {
                                pmd_flag = 0;
                                kprintf("                   \"pmd\": [\n");
                            }
                            kprintf("                       {\n");
                            kprintf("                           \"idx\": \"%ld\",\n", k);
                            kprintf("                           \"addr\": \"%p\",\n", pmd);
                            kprintf("                           \"value\": \"0x%lx\",\n", pmd_val(*pmd));
                            //kprintf("-------->pmd %p [%ld] = 0x%lx\n", pmd, k, pmd_val(*pmd));
                            pte = (pte_t *)(pmd_val(*pmd) );
                            if (((u64)pte &  PTE_TYPE_PAGE) == PTE_TYPE_PAGE) {
                                pte = (pte_t *)(pmd_val(*pmd) & PTE_ADDR_MASK );
                                for (l = 0; l < PTRS_PER_PTE; l++, pte++) 
                                {
                                    if (pte_val(*pte)) {
                                        if (pte_flag) {
                                            pte_flag = 0;
                                            kprintf("                           \"pte\": [\n");
                                        }
                                        kprintf("                               {\n");
                                        kprintf("                                   \"idx\": \"%ld\",\n", l);
                                        kprintf("                                   \"addr\": \"%p\",\n", pte);
                                        kprintf("                                   \"value\": \"0x%lx\",\n", pte_val(*pte));
                                        kprintf("                               },\n");
                                        //kprintf("------------>pte %p [%ld] = 0x%lx\n", pte, l, pte_val(*pte));
                                    }
                                }
                                kprintf("                           ],\n");
                                pte_flag = 1;
                            }
                            kprintf("                       },\n");
                        }
                    }
                    kprintf("                   ],\n");
                    kprintf("               },\n");
                    pmd_flag = 1;
                }
            }
            kprintf("           ],\n");
            kprintf("       },\n");
            pud_flag = 1;
        }
    }
    kprintf("    ],\n");
    kprintf("}\n");
}

unsigned long idmap_pgd[PTRS_PER_PGD];

#include "addressing.h"

int main()
{
    int a = 10;
    printf("hallo pgtable\n");

    memset(idmap_pgd, 0, sizeof(idmap_pgd));

    pgd_t *pgd = (pgd_t *)((u64)idmap_pgd);

    pg_map(pgd,  0x40000000,  0x90000000,  0x204000,  0);

    unsigned long pa = pgd_addressing((unsigned long *)pgd, 0x40003000);
    kprintf("pa = %p\n", pa);

    dump_pgtable_brief((pgd_t *)&idmap_pgd);

    return 0; 

}
