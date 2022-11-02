#include <stdlib.h>

#define VA_MASK ((1UL << 48) - 1) // VA_MASK = 0x0000_ffff_ffff_ffff
#define INDEX_MASK ((1UL << 9) - 1) // INDEX_MASK = 0x1ff
#define PAGE_MASK ((1UL << 12) - 1) // PAGE_MASK = 0xfff

//unsigned long va = 0xffff000010000000;
//unsigned long pgd[512];

static unsigned long pud_addressing(unsigned long *pud, unsigned long va);
static unsigned long pmd_addressing(unsigned long *pmd, unsigned long va);
static unsigned long pte_addressing(unsigned long *pmd, unsigned long va);

unsigned long pgd_addressing(unsigned long *pgd, unsigned long va)
{
    va &= VA_MASK; // va = 0x0000_0000_1000_1000

    // 获取L0索引
    unsigned int L0_index = (va  >> 39) & INDEX_MASK; //L0_index = 0x0000_0000_1000_1000 >> 39 & 0x1ff = 0

    // 获取PGD页表描述符
    unsigned long pgd_desc = pgd[L0_index];

    // 查看Table Descriptor Type位
    unsigned char desc_type = pgd_desc & 0x3;
    if (desc_type == 3) {
        // Type位为3代表，此描述符保存了下一级页表的地址，也就是PUD的地址，PUD的地址必须4K对齐
        unsigned long *pud = (unsigned long *)(pgd_desc & (~ 0x3));
        return pud_addressing(pud, va);
    } else if(desc_type == 1) {
        // 一级页表type必须为3
        abort();
    } else {
        abort();
    }
}

static unsigned long pud_addressing(unsigned long *pud, unsigned long va)
{
    unsigned long pa;
	// 获取L1索引
	unsigned int L1_index = (va >> 30) & INDEX_MASK; //L1_index = 0x0000_0000_1000_1000 >> 30 & 0x1ff = 0
    
    // 获取PUD页表描述符
	unsigned long pud_desc = pud[L1_index ];

    // 查看Table Descriptor Type位
	unsigned char desc_type = pud_desc & 0x3;
    if (desc_type == 3) {
    	// Type位为3代表，此描述符保存了下一级页表的地址，也就是PMD的地址，PMD的地址必须4K对齐
        unsigned long *pmd = (unsigned long *)(pud_desc & (~ 0x3));
        return pmd_addressing(pmd, va);
    } else if(desc_type == 1) {
        // Type位为1代表，此描述符保存的物理地址，PA地址必须1G对齐
        pa = pud_desc & (~ PAGE_MASK);
        unsigned long offset = va & ((1UL << 30) - 1);
        pa += offset;
    } else {
        abort();
    }
    return pa;
}

static unsigned long pmd_addressing(unsigned long *pmd, unsigned long va)
{
    unsigned long pa;
	// 获取L2索引
	unsigned int L2_index = (va >> 21) & INDEX_MASK; //L2_index = 0x0000_0000_1000_1000 >> 21 & 0x1ff = 0x80 & 0x1ff = 128
    
    // 获取PMD页表描述符
	unsigned long pmd_desc = pmd[L2_index];

    // 查看Table Descriptor Type位
	unsigned char desc_type = pmd_desc & 0x3;
    if (desc_type == 3) {
    	// Type位为3代表，此描述符保存了下一级页表的地址，也就是PTE的地址，PTE的地址必须4K对齐
        unsigned long *pte = (unsigned long *)(pmd_desc & (~ 0x3));
        return pte_addressing(pte, va);
    } else if(desc_type == 1) {
        // Type位为1代表，此描述符保存的物理地址，PA地址必须2M对齐
        pa = pmd_desc & (~ PAGE_MASK);
        unsigned long offset = va & ((1UL << 21) - 1);
        pa += offset;
    } else {
        abort();
    }
    return pa;
}

static unsigned long pte_addressing(unsigned long *pte, unsigned long va)
{
    unsigned long pa;
	// 获取L3索引
	unsigned int L3_index = (va >> 12) & INDEX_MASK; //L3_index = 0x0000_0000_1000_1000 >> 12 & 0x1ff = 1
    
    // 获取PMD页表描述符
	unsigned long pte_desc = pte[L3_index];

    // 查看Table Descriptor Type位
	unsigned char desc_type = pte_desc & 0x3;
    if (desc_type == 3) {
    	// Type位为3代表，此描述符保存的物理地址，PA地址必须4K对齐
        // PTE的Type位虽然为3，但并不代表还指向下一级，PTE已经是最后一级，描述的就是物理地址，当我map时把Type设置为1会出现错误，Type=1只有在PUD和PMD阶段起作用
        pa = pte_desc & (~ PAGE_MASK);
        unsigned long offset = va & ((1UL << 12) - 1);
        pa += offset;
    	abort();
    } else {
        abort();
    }
    return pa;
}