// clang++ -o2 --std=c++14 raw2jpg.cpp
// cl /Ox /MD /EHsc raw2jpg.cpp

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <cstdio>
#include <cstdint>
#include <vector>
#include <memory>
#include <stdexcept>

#include "zoomlogic.h"

// https://github.com/serge-rgb/TinyJPEG
#define TJE_IMPLEMENTATION
#include "tiny_jpeg.h"

#if defined(_WIN32)
#define binfp(m,fp) freopen(nullptr, m, fp)
#else
#define binfp(m,fp) (fp)
#endif

struct cfile
{
    void operator()(FILE* fp) const
    {
        if (fp != stdin && fp != stdout)
            fclose(fp);
    }
    using ptr = std::unique_ptr<FILE, cfile>;
};

const int rawBytesPerPixel = 2;
const int components = 3;

static void loadRawRgb16(std::vector<uint8_t>& dest, FILE* fpIn, int width, int height)
{
    auto srcStride = width * rawBytesPerPixel; // 16bit
    std::vector<uint8_t> src(srcStride);
    auto dstStride = width * components;
    auto size = dstStride * height;
    dest.resize(size);

    for (int c = 0; c < components; c++) // assuming RGB
    {
        for (int i = 0; i < height; i++)
        {
            auto p = &dest[i * dstStride + c];
            std::fread(&src[0], 1, srcStride, fpIn);
            auto q = &src[1]; // pointer to higher byte in little endian pixel value
            for (int j = 0; j < width; j++)
            {
                *p = *q;
                p += components;
                q += rawBytesPerPixel;
            }
        }
    }
}

static std::vector<uint8_t> loadRawRgb16(const char* fileName, int width, int height)
{
    cfile::ptr fpIn(std::strcmp(fileName, "-") ? std::fopen(fileName, "rb") : binfp("rb", stdin));
    if (!fpIn)
        throw std::runtime_error("Invalid input file name.");

    std::vector<uint8_t> raw;
    loadRawRgb16(raw, fpIn.get(), width, height);
    return raw;
}

static void fwrite_wrapper(void* context, void* data, int size) { fwrite(data, 1, size, reinterpret_cast<FILE*>(context)); }

int main(int argc, char* argv[])
{
    try
    {
        if (argc < 5)
        {
            fprintf(stderr, 
                "USAGE: raw2jpg INPUT OUTPUT WIDTH HEIGHT [OUTPUT_WIDTH OUTPUT_HEIGHT [QUALITY]]\n\n"
                "PARAMETERS:\n"
                "  INPUT   Input RAW file name or - to read from stdin.\n"
                "  OUTPUT  Output RAW file name or - to write out to stdout.\n"
            );
            return 1;
        }

        auto input = argv[1];
        auto output = argv[2];
        auto width = atoi(argv[3]);
        auto height = atoi(argv[4]);
        auto outWidth = argc > 6 ? atoi(argv[5]) : 0;
        auto outHeight = argc > 7 ? atoi(argv[6]) : 0;
        auto quality = argc > 8 ? atoi(argv[7]) * 3 / 100 : 2; // [0 100] -> [0 3]

        if (width <= 0 || height <= 0)
            throw std::runtime_error("Invalid width/height.\n");
        if (quality < 0 || quality > 3)
            throw std::runtime_error("quality value out of range [0 100].\n");

        auto raw = loadRawRgb16(input, width, height);

        cfile::ptr fpOut(std::strcmp(output, "-") ? std::fopen(output, "wb") : binfp("wb", stdout));
        if (!fpOut.get())
            throw std::runtime_error("Invalid output file name.\n");

        if (outWidth && outHeight)
        {
            auto outStride = outWidth * components;
            std::vector<uint8_t> buffer(outStride * outHeight);
            Zoom::rescale<components>(
                &buffer[0], outStride,
                0, 0, outWidth, outHeight, outWidth, outHeight,
                &raw[0], width * components,
                width, height);
            buffer.swap(raw);
            width = outWidth;
            height = outHeight;
        }

        auto ret = tje_encode_with_func(
            fwrite_wrapper, fpOut.get(),
            quality, width, height, components, &raw[0]);
        if (!ret)
            throw std::runtime_error("tje_encode_with_func failed.");
    }
    catch (std::runtime_error& e)
    {
        fprintf(stderr, "Error: %s\n", e.what());
        return -1;
    }
}
