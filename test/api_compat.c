// SPDX-License-Identifier: MIT
// Intentionally compiled as C17 to verify the C ABI of the C++ implementation.
#define _CRT_SECURE_NO_WARNINGS
#include "box2d/box2d.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "Failed: %s, line %d\n", #x, __LINE__); return 1; } } while (0)

int main(int argc, char** argv)
{
    b2WorldDef wd = b2DefaultWorldDef();
    wd.workerCount = 4;
    b2WorldId world = b2CreateWorld(&wd);
    CHECK(b2World_IsValid(world));
    if (argc == 3 && strcmp(argv[1], "--read") == 0)
    {
        char path[1024];
        snprintf(path, sizeof(path), "%s.snapshot", argv[2]);
        FILE* file = fopen(path, "rb");
        CHECK(file != NULL);
        CHECK(fseek(file, 0, SEEK_END) == 0);
        long size = ftell(file);
        CHECK(size > 0 && size < 100000000);
        rewind(file);
        unsigned char* bytes = malloc((size_t)size);
        CHECK(bytes != NULL);
        CHECK(fread(bytes, 1, (size_t)size, file) == (size_t)size);
        CHECK(fclose(file) == 0);
        CHECK(b2World_RestoreFromSnapshot(world, bytes, (int)size));
        free(bytes);
        printf("restored=%016" PRIx64 "\n", b2World_GetStateHash(world));
        for (int i = 0; i < 60; ++i) b2World_Step(world, 1.0f / 60.0f, 4);
        printf("continued=%016" PRIx64 "\n", b2World_GetStateHash(world));
        snprintf(path, sizeof(path), "%s.b2rec", argv[2]);
        b2Recording* rec = b2CreateRecordingFromFile(path);
        CHECK(rec != NULL);
        CHECK(b2ValidateRecording(b2Recording_GetData(rec), b2Recording_GetSize(rec), 4));
        b2DestroyRecording(rec);
    }
    else
    {
        b2BodyDef bd = b2DefaultBodyDef();
        b2BodyId ground = b2CreateBody(world, &bd);
        b2Polygon floor = b2MakeBox(30.0f, 0.5f);
        b2ShapeDef sd = b2DefaultShapeDef();
        b2CreatePolygonShape(ground, &sd, &floor);
        b2Polygon box = b2MakeBox(0.45f, 0.45f);
        for (int i = 0; i < 120; ++i)
        {
            bd.type = b2_dynamicBody;
            bd.position.x = (float)(i % 12) - 6.0f;
            bd.position.y = 1.1f + 1.0f * (float)(i / 12);
            b2BodyId body = b2CreateBody(world, &bd);
            b2CreatePolygonShape(body, &sd, &box);
        }
        b2Recording* rec = b2CreateRecording(4096);
        CHECK(rec != NULL);
        unsigned char* bytes = NULL;
        int size = 0;
        uint64_t snapshotHash = 0;
        b2World_StartRecording(world, rec);
        for (int i = 0; i < 240; ++i)
        {
            b2World_Step(world, 1.0f / 60.0f, 4);
            printf("%03d=%016" PRIx64 "\n", i, b2World_GetStateHash(world));
            if (i == 59)
            {
                size = b2World_GetSnapshot(world, NULL, 0);
                CHECK(size > 0);
                bytes = malloc((size_t)size);
                CHECK(bytes != NULL);
                CHECK(b2World_GetSnapshot(world, bytes, size) == size);
                snapshotHash = b2World_GetStateHash(world);
            }
        }
        b2World_StopRecording(world);
        CHECK(bytes != NULL);
        if (argc == 3 && strcmp(argv[1], "--write") == 0)
        {
            char path[1024];
            snprintf(path, sizeof(path), "%s.snapshot", argv[2]);
            FILE* file = fopen(path, "wb");
            CHECK(file != NULL);
            CHECK(fwrite(bytes, 1, (size_t)size, file) == (size_t)size);
            CHECK(fclose(file) == 0);
            snprintf(path, sizeof(path), "%s.b2rec", argv[2]);
            CHECK(b2Recording_SaveToFile(rec, path));
        }
        CHECK(b2World_RestoreFromSnapshot(world, bytes, size));
        CHECK(b2World_GetStateHash(world) == snapshotHash);
        CHECK(b2ValidateRecording(b2Recording_GetData(rec), b2Recording_GetSize(rec), 4));
        free(bytes);
        b2DestroyRecording(rec);
    }
    b2DestroyWorld(world);
    CHECK(b2GetByteCount() == 0);
    return 0;
}
