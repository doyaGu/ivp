#include <ivp_physics.hxx>
#include <ivp_great_matrix.hxx>

#include <cstdlib>

#define CHECK(condition) do { if (!(condition)) return 1; } while (0)

static void initialize_matrix(IVP_Great_Matrix_Many_Zero *matrix, int columns)
{
    matrix->columns = columns;
    matrix->calc_aligned_row_len();
    matrix->matrix_values = static_cast<IVP_DOUBLE *>(
        std::calloc(columns * matrix->aligned_row_len, sizeof(IVP_DOUBLE)));
    matrix->desired_vector = static_cast<IVP_DOUBLE *>(
        std::calloc(matrix->aligned_row_len, sizeof(IVP_DOUBLE)));
    matrix->result_vector = static_cast<IVP_DOUBLE *>(
        std::calloc(matrix->aligned_row_len, sizeof(IVP_DOUBLE)));
}

static void destroy_matrix(IVP_Great_Matrix_Many_Zero *matrix)
{
    std::free(matrix->matrix_values);
    std::free(matrix->desired_vector);
    std::free(matrix->result_vector);
    matrix->matrix_values = NULL;
    matrix->desired_vector = NULL;
    matrix->result_vector = NULL;
}

int main()
{
    IVP_Great_Matrix_Many_Zero big;
    IVP_Great_Matrix_Many_Zero sub;
    initialize_matrix(&big, 3);
    initialize_matrix(&sub, 2);

    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            big.matrix_values[row * big.aligned_row_len + column] =
                row * 3 + column + 1;
        }
    }

    IVP_DOUBLE packed[9];
    IVP_DOUBLE desired[3];
    big.copy_matrix(packed, desired);
    int positions[2] = {0, 2};
    big.copy_to_sub_matrix(packed, &sub, positions);

    CHECK(sub.matrix_values[0] == 1.0);
    CHECK(sub.matrix_values[1] == 3.0);
    CHECK(sub.matrix_values[sub.aligned_row_len] == 7.0);
    CHECK(sub.matrix_values[sub.aligned_row_len + 1] == 9.0);

    destroy_matrix(&big);
    destroy_matrix(&sub);
    return 0;
}
