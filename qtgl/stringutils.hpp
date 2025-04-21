#pragma once

#include <string>
#include <vector>

namespace qtgl {

std::vector<unsigned char> base64_decode(const std::string& encoded_string) {
  const std::string base64_chars =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int decoding_table[256] = {0};
  for (int i = 0; i < 64; i++) decoding_table[static_cast<unsigned char>(base64_chars[i])] = i;

  std::vector<int> filtered;
  std::vector<bool> is_padding;
  for (char c : encoded_string) {
    unsigned char uc = static_cast<unsigned char>(c);
    if (uc == '=') {
      filtered.push_back(0);
      is_padding.push_back(true);
    } else {
      int val = decoding_table[uc];
      if (val != 0 || uc == 'A') {  // 'A'解码为0，需特殊处理
        filtered.push_back(val);
        is_padding.push_back(false);
      }
    }
  }

  size_t filtered_len = filtered.size();
  if (filtered_len % 4 != 0) return {};

  std::vector<unsigned char> decoded_data;
  for (size_t i = 0; i < filtered_len; i += 4) {
    int quad[4] = {filtered[i], filtered[i + 1], filtered[i + 2], filtered[i + 3]};
    bool padding[4] = {is_padding[i], is_padding[i + 1], is_padding[i + 2], is_padding[i + 3]};

    int padding_count = 0;
    for (int j = 0; j < 4; j++) {
      if (padding[j]) {
        if (j < 2) return {};
        padding_count++;
      }
    }
    if ((padding[2] && !padding[3]) || padding_count > 2) return {};

    int bytes = 3 - padding_count;
    uint32_t triple = (quad[0] << 18) | (quad[1] << 12) | (quad[2] << 6) | quad[3];

    decoded_data.push_back((triple >> 16) & 0xFF);
    if (bytes > 1) decoded_data.push_back((triple >> 8) & 0xFF);
    if (bytes > 2) decoded_data.push_back(triple & 0xFF);
  }

  return decoded_data;
}

}  // namespace qtgl