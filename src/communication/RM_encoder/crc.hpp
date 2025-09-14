#pragma once
#include <stdint.h>

namespace __detail
{

// CRC8部分
// crc8 generator polynomial:G(x)=x8+x5+x4+1
unsigned char CRC8;
unsigned char CRC8_INIT = 0xff;
uint16_t CRC16;

const unsigned char CRC8_TAB[ 256 ] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35 };

/**
 * @brief 计算输入数据的CRC8校验码
 * @param pchMessage 输入数据指针
 * @param dwLength 输入数据长度
 * @param ucCRC8 初始CRC8值，通常是0xFF
 * @return 计算得到的CRC8校验码
 *
 * 该函数基于CRC8查找表迭代设计，适合于流式数据校验，通过异或和查表计算校验值
 */
unsigned char CRC_GetCRC8CheckSum(unsigned char *pchMessage,
                                  unsigned int dwLength, char ucCRC8)
{
    unsigned char ucIndex = 0; // 查表索引

    while (dwLength--)
    {
        // 使用当前CRC值与数据字节异或得到查表索引
        ucIndex = ucCRC8 ^ (*pchMessage++);
        // 利用查表获取新的CRC值
        ucCRC8 = CRC8_TAB[ucIndex];
    }

    return ucCRC8;
}

/**
 * @brief 在数据末尾添加CRC8校验码
 * @param pchMessage 数据缓冲区（包含预留空间放校验码）
 * @param dwLength 缓冲区总长度（数据+校验码位数）
 * 
 * 该函数计算前dwLength-1字节数据的CRC8，结果写入最后1字节
 */
void CRC_AppendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;

    // 参数校验，指针不能为空，且数据长度至少大于2（至少有1字节数据+1字节校验码）
    if ((pchMessage == nullptr) || (dwLength <= 2))
        return;

    // 计算CRC8校验码，初始值为CRC8_INIT
    ucCRC = CRC_GetCRC8CheckSum(pchMessage, dwLength - 1, CRC8_INIT);

    // 将计算得到的CRC8码写到数据末尾（最后一个字节）
    pchMessage[dwLength - 1] = ucCRC;
}

// ------------------------------------------------------
// CRC16相关函数
// ------------------------------------------------------

/**
 * @brief 计算输入数据的CRC16校验码
 * @param pchMessage 输入数据指针
 * @param dwLength 输入数据长度
 * @param wCRC 初始CRC16值，通常为0xFFFF
 * @return 计算得到的CRC16校验码（16位无符号整数）
 *
 * 该函数通常依赖预先计算好的CRC16查找表，参数wCRC允许多段数据连续计算保持状态
 * 若使用不同的CRC16算法，CRC16_Table应替换为对应多项式的表
 */
uint16_t CRC_GetCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData = 0;

    if (pchMessage == nullptr)
        return 0xffff;  // 输入空指针，返回0xFFFF作为错误标志或无效CRC

    while (dwLength--)
    {
        chData = *pchMessage++; // 当前数据字节
        // 先将CRC高8位右移，之后与低8位异或当前字节后查表，得到新CRC值
        wCRC = (wCRC >> 8) ^
               CRC16_Table[((wCRC ^ chData) & 0x00ff)];
    }

    return wCRC;
}

/**
 * @brief 在数据末尾添加CRC16校验码
 * @param pchMessage 数据缓冲区（包括预留空间用于CRC16 2字节）
 * @param dwLength 缓冲区总长度（数据长度+2字节CRC）
 *
 * 计算前dwLength-2字节数据的CRC16，结果写入最后2字节
 */
void CRC_AppendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;

    // 参数校验，指针不能为空，且长度至少大于2（至少有1字节数据+2字节CRC）
    if ((pchMessage == nullptr) || (dwLength <= 2))
        return;

    // 计算CRC16校验码，使用全局CRC16_INIT初始值
    wCRC = CRC_GetCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);

    // 校验码低8位放倒数第二个字节
    pchMessage[dwLength - 2] = static_cast<uint8_t>(wCRC & 0x00ff);
    // 校验码高8位放最后一个字节
    pchMessage[dwLength - 1] = static_cast<uint8_t>((wCRC >> 8) & 0x00ff);
}

} // namespace __detail

// ------------------------------------------------------
// CRC8校验验证函数
// ------------------------------------------------------

/**
 * @brief 校验数据末尾CRC8是否匹配
 * @param pchMessage 待校验数据（包括数据+1字节CRC）
 * @param dwLength 数据长度（包含CRC字节）
 * @return 1匹配成功 0不匹配
 *
 * 逻辑：重新计算前（dwLength-1）个字节CRC8，判断是否与最后一个字节相等
 */
unsigned int CRC_VerifyCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;

    if ((pchMessage == nullptr) || (dwLength <= 2))
        return 0; // 输入非法，直接判定不匹配

    ucExpected = __detail::CRC_GetCRC8CheckSum(pchMessage, dwLength - 1, __detail::CRC8_INIT);

    return (ucExpected == pchMessage[dwLength - 1]) ? 1 : 0;
}

// ------------------------------------------------------
// CRC16校验验证函数
// ------------------------------------------------------

/**
 * @brief 校验数据末尾CRC16是否匹配
 * @param pchMessage 待校验数据（包括数据+2字节CRC）
 * @param dwLength 数据长度（包含CRC字节）
 * @return 1匹配成功 0不匹配
 *
 * 逻辑：重新计算前（dwLength-2）个字节CRC16，
 *       判断低8位是否等于倒数第二字节，且高8位是否等于倒数第一个字节
 */
uint32_t CRC_VerifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;

    if ((pchMessage == nullptr) || (dwLength <= 2))
        return 0; // 输入非法，直接判定不匹配

    wExpected = __detail::CRC_GetCRC16CheckSum(pchMessage, dwLength - 2, __detail::CRC16_INIT);

    // 比较计算得到的校验码和数据中附带的校验码是否一致
    return ((wExpected & 0xff) == pchMessage[dwLength - 2]) &&
           (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]) ? 1 : 0;
}