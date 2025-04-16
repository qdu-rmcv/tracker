// // Copyright (C) 2022 ChenJun
// // Copyright (C) 2024 Zheng Yu
// // Licensed under the Apache-2.0 License.

// #ifndef RM_SERIAL_DRIVER__CRC_HPP_
// #define RM_SERIAL_DRIVER__CRC_HPP_

// #include <cstdint>

// namespace crc16
// {
// /**
//   * @brief CRC16 Verify function
//   * @param[in] pchMessage : Data to Verify,
//   * @param[in] dwLength : Stream length = Data + checksum
//   * @return : True or False (CRC Verify Result)
//   */
// uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

// /**
//   * @brief Append CRC16 value to the end of the buffer
//   * @param[in] pchMessage : Data to Verify,
//   * @param[in] dwLength : Stream length = Data + checksum
//   * @return none
//   */
// void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

// }  // namespace crc16

// #endif  // RM_SERIAL_DRIVER__CRC_HPP_

// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>

namespace crc16
{
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);//验证
inline uint16_t CRC16_Byte(uint16_t crc, const uint8_t data); //ji suan crc
uint16_t CRC16_Calc(const uint8_t *buf, std::size_t len, uint16_t crc);//jisuan crc
bool CRC16_Verify(const uint8_t *buf, std::size_t len); //pan duan
/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,要验证的数据
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);//追加

}  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_

