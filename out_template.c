#include<stdio.h>
#define TEMPLATE_FILE_NAME "template.txt"

int main()
{
    printf("[START]\n\n");

    FILE *f = fopen(TEMPLATE_FILE_NAME, "r");
    if(f == NULL)
    {
        f = fopen(TEMPLATE_FILE_NAME, "w");
        if(f != NULL)
        {
            fprintf(f,
                "X_Mesh_Sizes:1, 2, 3\n"
                "Y_Mesh_Sizes:1, 2, 3\n"
                "Z_Mesh_Sizes:1, 2, 3\n"
                "Column : Span() Width() Depth() X_Center() Y_Center()\n"
                "Beam   : Span() Width() Depth() Y_Center() Z_Center()\n"
                "xBeam  : Width()\n"
                "Rebar 1: X=1 Y=1\n"
                "Rebar 2: X=1 Y=1\n"
                "Rebar 3: X=1 Y=1\n"
                "END\n");
            fclose(f);
        }
        else
        {
            printf("[ERROR] file cant open\n");
            return 1;
        }
    }
    else
    {
        printf(TEMPLATE_FILE_NAME " already exist\n");
        fclose(f);
    }
    printf("\n[END]\n");
    return 0;
}