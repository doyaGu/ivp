#include <ivp_physics.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_surbuild_pointsoup.hxx>
#include <ivu_min_hash.hxx>

#include <cstdio>
#include <cmath>

#ifdef IVP_TEST_QHULL_PARALLEL
#include <atomic>
#include <thread>
#endif

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
    return false; } } while (0)

static bool check_ledge(IVP_Compact_Ledge *ledge, int point_count, int triangle_count)
{
    CHECK(ledge);
    const int count = (ledge->get_size() - sizeof(IVP_Compact_Ledge)
        - ledge->get_n_triangles() * sizeof(IVP_Compact_Triangle))
        / sizeof(IVP_Compact_Poly_Point);
    CHECK(count == point_count);
    CHECK(ledge->get_n_triangles() == triangle_count);
    const IVP_Compact_Triangle *tri = ledge->get_first_triangle();
    for (int i = 0; i < triangle_count; ++i, ++tri)
    {
        for (int j = 0; j < 3; ++j)
        {
            const IVP_Compact_Edge *edge = tri->get_edge(j);
            CHECK(edge->get_start_point_index() < point_count);
            CHECK(edge->get_opposite()->get_opposite() == edge);
            CHECK(edge->get_opposite()->get_start_point_index()
                == edge->get_next()->get_start_point_index());
        }
    }
    return true;
}

static bool build_cube(int variant)
{
    IVP_U_Point vertices[10];
    IVP_U_Vector<IVP_U_Point> points;
    const double shift = variant % 13;
    const double scale = 1.0 + (variant % 7) * 0.25;
    for (int i = 0; i < 8; ++i)
    {
        vertices[i].set(shift + ((i & 1) ? scale : -scale),
                        ((i & 2) ? scale : -scale),
                        ((i & 4) ? scale : -scale));
        points.add(&vertices[i]);
    }
    vertices[8].set(shift, 0, 0); // interior point
    vertices[9].set(&vertices[0]); // duplicate corner
    points.add(&vertices[8]);
    points.add(&vertices[9]);
    IVP_Compact_Ledge *ledge =
        IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
#ifdef IVP_TEST_QHULL_ENABLED
    CHECK(check_ledge(ledge, 8, 12));
    // Every retained vertex must keep the original coordinates.
    const IVP_Compact_Poly_Point *array = ledge->get_point_array();
    unsigned int found = 0;
    for (int i = 0; i < 8; ++i)
    {
        int match = -1;
        for (int j = 0; j < 8; ++j)
        {
            if (std::fabs(array[i].k[0] - vertices[j].k[0]) < 1e-6
                && std::fabs(array[i].k[1] - vertices[j].k[1]) < 1e-6
                && std::fabs(array[i].k[2] - vertices[j].k[2]) < 1e-6)
                match = j;
        }
        CHECK(match >= 0);
        found |= 1u << match;
    }
    CHECK(found == 255);
    ivp_free_aligned(ledge);
#else
    CHECK(ledge == NULL);
#endif
    return true;
}

#ifndef IVP_TEST_QHULL_PARALLEL
static bool run_serial()
{
    // Interleaved heaps must retain their own insertion order for equal keys.
    IVP_U_Min_Hash first_hash(8), second_hash(8);
    int keys[8];
    for (int i = 0; i < 4; ++i)
    {
        first_hash.add(&keys[i], 1.0);
        second_hash.add(&keys[7 - i], 1.0);
    }
    for (int i = 0; i < 4; ++i)
    {
        CHECK(first_hash.find_min_elem() == &keys[i]);
        CHECK(second_hash.find_min_elem() == &keys[7 - i]);
        first_hash.remove_min();
        second_hash.remove_min();
    }
    IVP_U_Point vertices[4];
    IVP_U_Vector<IVP_U_Point> points;
    vertices[0].set(0, 0, 0);
    vertices[1].set(1, 0, 0);
    vertices[2].set(0, 1, 0);
    vertices[3].set(0, 0, 1);
    for (int i = 0; i < 3; ++i)
        points.add(&vertices[i]);
    IVP_Compact_Ledge *ledge =
        IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
    CHECK(check_ledge(ledge, 3, 2)); // triangle path also works with Qhull disabled
    ivp_free_aligned(ledge);
    points.add(&vertices[3]);
    ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
#ifdef IVP_TEST_QHULL_ENABLED
    CHECK(check_ledge(ledge, 4, 4));
    ivp_free_aligned(ledge);
#else
    CHECK(ledge == NULL);
#endif
    // Repeated failed hulls exercise cleanup and the joggled retry path.
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        for (int i = 0; i < 4; ++i)
            vertices[i].set(0, 0, 0);
        CHECK(IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points) == NULL);
        CHECK(build_cube(attempt));
    }
    for (int i = 0; i < 32; ++i)
        CHECK(build_cube(i));
    IVP_SurfaceBuilder_Pointsoup::cleanup();
    return true;
}
#endif

int main()
{
#ifdef IVP_TEST_QHULL_PARALLEL
    const int worker_count = 4;
    std::atomic<int> ready(0);
    std::atomic<bool> start(false);
    std::atomic<bool> success(true);
    std::thread workers[worker_count];
    for (int i = 0; i < worker_count; ++i)
    {
        workers[i] = std::thread([&, i]() {
            ready.fetch_add(1);
            while (!start.load())
                std::this_thread::yield();
            for (int j = 0; j < 100; ++j)
            {
                if (!build_cube(i * 100 + j))
                {
                    success.store(false);
                    break;
                }
            }
        });
    }
    while (ready.load() != worker_count)
        std::this_thread::yield();
    start.store(true);
    for (int i = 0; i < worker_count; ++i)
        workers[i].join();
    return success.load() ? 0 : 1;
#else
    return run_serial() ? 0 : 1;
#endif
}
