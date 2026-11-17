#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <openssl/evp.h>

namespace sc_cache {
using Digest = std::array<unsigned char, 32>;

class Hash {
public:
    Hash() : context_(EVP_MD_CTX_new(), EVP_MD_CTX_free) {
        if (!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1)
            throw std::runtime_error("Cannot initialize SC cache digest");
    }
    void add(const void *data, size_t bytes) {
        if (EVP_DigestUpdate(context_.get(), data, bytes) != 1)
            throw std::runtime_error("Cannot update SC cache digest");
    }
    Digest finish() {
        Digest result{};
        unsigned int length = 0;
        if (EVP_DigestFinal_ex(context_.get(), result.data(), &length) != 1 || length != result.size())
            throw std::runtime_error("Cannot finish SC cache digest");
        return result;
    }
private:
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_;
};

inline Digest fileDigest(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open map for cache identity");
    Hash hash;
    char block[65536];
    while (input.read(block, sizeof block) || input.gcount()) hash.add(block, input.gcount());
    if (!input.eof()) throw std::runtime_error("Cannot read map for cache identity");
    return hash.finish();
}

// Field-by-field encoding (no struct padding). Native byte order is explicitly
// checked; a cache from a different ABI/algorithm is rebuilt, never interpreted.
inline constexpr std::array<char, 8> magic{'G','S','S','C','C','0','0','2'};
inline constexpr uint32_t version = 2, endian = 0x01020304;
inline constexpr uint64_t headerBytes = 112, digestOffset = 80;
inline constexpr uint64_t maxBytes = 256ULL * 1024 * 1024;
}  // namespace sc_cache
