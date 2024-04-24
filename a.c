#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#define OUT_FILE_NAME      "out.ffi"
#define OPTCHAR            "bfo:"
#define MESH_ARRAY_MAX 64 //注意が必要
#define REBAR_MAX      16 
#define BOUNDARY_MAX   8 //確定
#define XYZ            3    

typedef struct components
{
    double  span;
    double  width;
    double  depth;
    double  center[XYZ];
    double  Fc;
} Components;

typedef struct mainRebar 
{
    double cordi[2];
} MainRebar;

typedef struct modelsize
{
    Components column;
    Components beam;
    Components xbeam;
    MainRebar  rebar[REBAR_MAX];
} ModelSize;

typedef struct _mesh
{
    int    number; //メッシュ配列の要素数
    double length  [MESH_ARRAY_MAX];
} Mesh;

typedef struct geometry
{
    Mesh mesh;
    int  boundary[BOUNDARY_MAX];
} Geometry;

typedef struct nodeElm 
{
    int node;
    int elm;
} NodeElm;

typedef struct increment
{
    int node[XYZ];
    int elm [XYZ];
} Increment;

typedef struct startEnd
{
    int start[XYZ];
    int end  [XYZ];
} StartEnd;

typedef struct loadNode
{
    int node1;
    int node2;
} LoadNode;

void read_csv(FILE *f, Geometry *geo)
{
    int cnt = 0;
    if(fgetc(f) == '(')
    {
        do
        {
            fscanf(f, "%lf[^,)]", &geo->mesh.length[cnt++]);
        } while (fgetc(f) != ')');
        geo->mesh.number = cnt;
        geo->mesh.length[cnt] = -999.0;
    }
}

int load_inputdata(const char *inputFileName, Geometry geo[], ModelSize *modelSize)
{
    //メッシュ
    char str[3][16] = {"X_Mesh_Sizes", "Y_Mesh_Sizes", "Z_Mesh_Sizes"};
    FILE *f = fopen(inputFileName, "r");
    if(f == NULL){
        printf("[ERROR] %s cant open.\n", inputFileName);
        return 1;
    }
    
    int ch;
    for(int i = 0; i < 3; i++)
    {
        int cnt = 0;
        while(str[i][cnt] != '\0')
        {
            ch = fgetc(f);
            if(ch != str[i][cnt++])
            {
                fprintf(stderr, "[ERROR]\n");
                return 1;
            }
        }
        read_csv(f, &geo[i]);
        while((fgetc(f)) != '\n'){}
    }
    //柱梁
    fscanf(f, "Column : Span(%lf) Width(%lf) Depth(%lf) X_Center(%lf) Y_Center(%lf) Fc(%lf)",
        &modelSize->column.span, &modelSize->column.width, &modelSize->column.depth, &modelSize->column.center[0], &modelSize->column.center[1], &modelSize->column.Fc);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "Beam   : Span(%lf) Width(%lf) Depth(%lf) Y_Center(%lf) Z_Center(%lf)",
        &modelSize->beam.span, &modelSize->beam.width, &modelSize->beam.depth, &modelSize->beam.center[1], &modelSize->beam.center[2]);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "xBeam  : Width(%lf)",&modelSize->xbeam.width);
    while((fgetc(f)) != '\n'){}
    //主筋
    int index;
    fscanf(f, "Rebar %d:", &index);
    while(index <= REBAR_MAX)
    {
        fscanf(f, "Rebar %d:", &index);
        int result = fscanf(f, " X(%lf) Y(%lf)", &modelSize->rebar[index - 1].cordi[0], &modelSize->rebar[index - 1].cordi[1]);
        if (result == EOF) {
            // ファイルの終わりに達した場合はループを終了
            break;
        } else if (result == 0) {
            // 数値の読み取りに失敗した場合はエラーを表示して終了
            modelSize->rebar[index].cordi[0] = -999;
            modelSize->rebar[index].cordi[1] = -999;
            break;
        }
        modelSize->rebar[index - 1].cordi[0] += (modelSize->beam.span - modelSize->column.depth) / 2;
        while((fgetc(f)) != '\n'){}
    }
    fclose(f);
    return 0;
}

/*mesh範囲を全体に広げる*/
void symmetrical_input(struct geometry *mesh)
{
    int original_number = mesh->mesh.number;
    for (int i = 0; i < original_number; i++) {
        mesh->mesh.length[2 * original_number - i - 1] = mesh->mesh.length[i];
    }
    mesh->mesh.number *= 2;
}

void get_load_node(LoadNode *load, int startNode, const int pp[], const Geometry geo[])
{
    load->node1 = startNode  + (geo[0].boundary[3] - geo[0].boundary[2]) * pp[0] + (geo[1].boundary[2] - geo[1].boundary[0]) * pp[1];
    load->node2 = load->node1 + (geo[2].boundary[5] + 4) * pp[2];
}

int coordinate_to_point(double cordi, const double array[])
{
    double sum = 0;
    for(int i = 0; i < MESH_ARRAY_MAX; i++)
    {
        sum += array[i];
        if(fabs(sum - cordi) < 0.001)
        {
            return i + 1;
        }
        else if(sum > cordi)
        {
            printf("[ERROR] coordinate_to_point\n");
            return -999;
        }
    }
    return -999;
}

double point_to_coordinate(int point, const double array[])
{
    double cordi = 0;
    for(int i = 0; i < point; i++)
    {
        cordi += array[i];
    }
    return cordi;
}

int count_consecutive(int start, int end, const double array[])
{
    if(start > end)
    {
        printf("[ERROR] count_consecutive\n");
        return -1;
    }
    int count = 1;
    for(int i = start + 1; i < end; i++)
    {
        if(fabs(array[i - 1] - array[i]) < 0.001)
        {
            count++;
            continue;
        }
        else
        {
            break;
        }
    }
    return count;
}

void get_boundary(struct modelsize rcs, struct geometry model[])
{
    //x
    model[0].boundary[0] = 0;
    model[0].boundary[1] = 1;
    model[0].boundary[2] = coordinate_to_point((rcs.beam.span - rcs.column.depth) / 2, model[0].mesh.length);
    model[0].boundary[3] = model[0].mesh.number / 2;
    model[0].boundary[4] = coordinate_to_point((rcs.beam.span + rcs.column.depth) / 2, model[0].mesh.length);
    model[0].boundary[5] = model[0].mesh.number - 1;
    model[0].boundary[6] = model[0].mesh.number;
    //y
    model[1].boundary[0] = 0;
    model[1].boundary[1] = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, model[1].mesh.length);
    model[1].boundary[2] = model[1].mesh.number;
    //z
    model[2].boundary[0] = 0;
    model[2].boundary[1] = 1;
    model[2].boundary[2] = coordinate_to_point((rcs.column.span - rcs.beam.depth) / 2, model[2].mesh.length);
    model[2].boundary[3] = coordinate_to_point((rcs.column.span + rcs.beam.depth) / 2, model[2].mesh.length);
    model[2].boundary[4] = model[2].mesh.number - 1;
    model[2].boundary[5] = model[2].mesh.number;
}

void get_increment(Increment *column, Increment *beam, const Geometry geo[])
{
    //柱
    column->node[0] = 1;
    column->node[1] = geo[0].boundary[4] - geo[0].boundary[2] + 2;
    column->node[2] = column->node[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 1);
    column->elm [0] = 1;
    column->elm [1] = geo[0].boundary[4] - geo[0].boundary[2];
    column->elm [2] = column->elm[1] * (geo[1].boundary[2] - geo[1].boundary[0]);
    //梁
    beam->node[0] = 1;
    beam->node[1] = geo[0].boundary[6] + 1;
    beam->node[2] = beam->node[1] * (geo[1].boundary[2] - geo[1].boundary[1] + 1);
    beam->elm [0] = 1;
    beam->elm [1] = geo[0].boundary[6];
    beam->elm [2] = beam->elm[1] * (geo[1].boundary[2] - geo[1].boundary[1] + 1);
}

int next_index(int last_node)
{
    for(int j = 6; j >= 0; --j) //10の6乗から調べているから十分
    {
        int one = 1;
        for(int i = 0; i < j; ++i)
        {
            one *= 10;
        }
        if(last_node / one != 0)
        {
            return (last_node / one + 1) * one + 1;
        }
    }
    return 0;
}

void set_start_end(struct startEnd *point, int start, int end, int boundary[], int dir)
{
    point->start[dir] = boundary[start];
    point->end  [dir] = boundary[end];
}

void console_model_sizes(struct modelsize rcs)
{
    printf("\n------------------[ MODEL ]------------------\n\n");
    printf("           span      width     depth     center\n");
    printf("column   | %7.2f | %7.2f | %7.2f |x %7.2f |y %7.2f \n", rcs.column.span, rcs.column.width, rcs.column.depth, rcs.column.center[0], rcs.column.center[1]);
    printf("beam     | %7.2f | %7.2f | %7.2f |y %7.2f |z %7.2f \n", rcs.beam.span, rcs.beam.width, rcs.beam.depth, rcs.beam.center[1], rcs.beam.center[2]);
    printf("xbeam    |         | %7.2f |         |          |     \n\n", rcs.xbeam.width);
    
    printf("reber");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf(" %7d ", i + 1);
    }
    printf("\nX    ");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf(" %7.2f ", rcs.rebar[i].cordi[0]);
    }
    printf("\nY    ");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf(" %7.2f ", rcs.rebar[i].cordi[1]);
    }
    printf("\n");

}

void console_mesh(struct geometry modelGeometry[])
{
    int dir[3] = {'x', 'y', 'z'};
    printf("\n------------------[ MESH ]------------------\n");
    for(int j = 0; j < 3; j++)
    {
        printf("\n%c mesh count --> %d\n", dir[j], modelGeometry[j].mesh.number);
        printf("  i :");
        for(int i = 0; i < modelGeometry[j].mesh.number; i++)
        {
            printf(" %6d ", i);
        }
        printf("\nlen :");
        for(int i = 0; i < modelGeometry[j].mesh.number; i++)
        {
            printf(" %6.2f ", modelGeometry[j].mesh.length[i]);
        }
    }
}

void console_boundary(struct geometry model[])
{
    printf("\n------------------[ BOUNDARY ]------------------\n");
    printf("X  : ");
    for(int i = 0; i < 7; i++)
    {
        printf(" %4d ", model[0].boundary[i]);
    }
    printf("\nY  : ");
    for(int i = 0; i < 3; i++)
    {
        printf(" %4d ", model[1].boundary[i]);
    }
    printf("\nZ  : ");
    for(int i = 0; i < 6; i++)
    {
        printf(" %4d ", model[2].boundary[i]);
    }
    printf("\n");
}

void console_increment(struct increment pp)
{
    printf("\n------------------[ INCREMENT ]------------------\n");
    printf("         X     Y     Z\n");
    printf("node %5d %5d %5d\n", pp.node[0], pp.node[1], pp.node[2]);
    printf("elm  %5d %5d %5d\n", pp.elm[0], pp.elm[1], pp.elm[2]);
}

void console_index(struct nodeElm index, char *str)
{
    printf("\n------------------[ INDEX ]------------------\n");
    printf("%s\n", str);
    printf("node : %5d\n", index.node);
    printf("elm  : %5d\n", index.elm);
} 

/*--データ書き込み-------------------------------------------------*/
//頭のひな形
void print_head_template(FILE *f, struct loadNode load)
{
    fprintf(f,
    "-------------------< FINAL version 11  Input data >---------------------\n"
    "TITL :\n"
    "EXEC :STEP (    1)-->(   11)  ELASTIC=( ) CHECK=(1) POST=(1) RESTART=( )\n"
    "LIST :ECHO=(0)  MODEL=(1)  RESULTS=(1)  MESSAGE=(2)  WARNING=(2)  (0:NO)\n"
    "FILE :CONV=(2)  GRAPH=(2)  MONITOR=(2)  HISTORY=(1)  ELEMENT=(0)  (0:NO)\n"
    "DISP :DISPLACEMENT MONITOR NODE NO.(%5d)  DIR=(1)    FACTOR=\n"
    "LOAD :APPLIED LOAD MONITOR NODE NO.(%5d)  DIR=(1)    FACTOR=\n"
    "UNIT :STRESS=(3) (1:kgf/cm**2  2:tf/m**2  3:N/mm**2=MPa)\n\n", load.node2, load.node2);
}

void column_axial_force(FILE *f, int startElm, struct geometry geo[], double Fc, int elmPp[])
{
    int elm;
    double rate = 0.2;
    double unit = rate * Fc;
    fprintf(f,"\nSTEP :UP TO NO.(    1)   MAXIMUM LOAD INCREMENT=         CREEP=( )(0:NO)\n");
    for(int i = 0; i < geo[1].boundary[2]; i++)
    {
        elm = startElm + i * elmPp[1];
        fprintf(f,"  UE :ELM   S(%5d)-E(%5d)-I(%5d)     UNIT=%-9.4fDIR(3)  FACE(1)\n", elm, elm + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * elmPp[0], elmPp[0], unit);
    }
    for(int i = 0; i < geo[1].boundary[2]; i++)
    {
        elm = startElm + i * elmPp[1] + (geo[2].boundary[4] - geo[2].boundary[1]) * elmPp[2];
        fprintf(f,"  UE :ELM   S(%5d)-E(%5d)-I(%5d)     UNIT=-%-8.4fDIR(3)  FACE(2)\n", elm, elm + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * elmPp[0], elmPp[0], unit);
    }
    fprintf(f," OUT :STEP  S(    1)-E(     )-I(     ) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n\n");
}

void print_tail_template(FILE *f, struct loadNode load, int startElm, struct geometry geo[], double Fc, int elmPp[])
{
    fprintf(f,
    "\n----+----\n"
    "TYPH :(  1)  MATS(  1)  AXIS(   )\n"
    "TYPH :(  2)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  3)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  4)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  5)  MATC(  1)  AXIS(   )\n"
    "TYPB :(  1)  MATS(  2)  AXIS(   )  COEF=        D1=1       D2=        :\n"
    "TYPQ :(  1)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  2)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  3)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  4)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  5)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  6)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  7)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  8)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :(  9)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :( 10)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :( 11)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :( 12)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :( 13)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    /*"TYPQ :( 14)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"
    "TYPQ :( 15)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "  LQ :  Z-LAYER=( 4)  OFFSET=        TR-SHEAR=(2) (0:NO)  Z-ROTATION=(0)\n"*/
    "TYPL :(  1)  MATJ(  1)  AXIS(  1)  THICKNESS=1        Z=(1) (1:N  2:S)\n"
    "TYPF :(  1)  MATJ(  2)  AXIS(   )\n"
    "\n----+----\n"
    "MATS :(  1)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATS :(  2)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATC :(  1)  EC=2.05   (E+4) PR=0.2   FC=30     FT=3     ALP=      (E-5)\n"
    "MATJ :(  1)  TYPE=(4) (1:CRACK  2:BOND  3:GENERIC  4:RIGID  5:DASHPOT)\n"
    "MATJ :(  2)  TYPE=(4) (1:CRACK  2:BOND  3:GENERIC  4:RIGID  5:DASHPOT)\n"
    "\n----+----\n"
    "AXIS :(  1)  TYPE=(1) (1:GLOBAL 2:ELEMENT 3:INPUT 4:CYLINDER 5:SPHERE)\n"
    "\n----+----");
    column_axial_force(f, startElm, geo, Fc, elmPp);
    fprintf(f,
    "STEP :UP TO NO.(   11)   MAXIMUM LOAD INCREMENT=         CREEP=(0)(0:NO)\n"
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=-10      DIR(1)\n"
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=10       DIR(1)\n"
    " OUT :STEP  S(    2)-E(   11)-I(    1) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n"
    "\nEND\n", load.node1, load.node2);
}

/*NODEデータ書き込み*/
void print_NODE(FILE *f, int node, const double coordinates[]) 
{
    fprintf(f, "NODE :(%5d)  X=%-10.2fY=%-10.2fZ=%-10.2fRC=(000000)\n", node, coordinates[0], coordinates[1], coordinates[2]);
}

/*COPY:NODEデータ書き込み*/
void print_COPYNODE(FILE *f, int start, int end, int interval, double meshLen, int increment, int set, int dir) 
{
    char _end[6]      = " ";
    char _interval[6] = " ";

    if (end != 0)
    {
        sprintf(_end, "%d", end);
    }

    if (interval != 0)
    {
        sprintf(_interval, "%d", interval);
    }

    if (set <= 0)
     {} 
    else 
    {
        switch (dir) 
        {
            case 0  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DX=%-9.2lfINC(%5d)-SET(%4d)\n", start, _end, _interval, meshLen, increment, set); break;
            case 1  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DY=%-9.2lfINC(%5d)-SET(%4d)\n", start, _end, _interval, meshLen, increment, set); break;
            case 2  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DZ=%-9.2lfINC(%5d)-SET(%4d)\n", start, _end, _interval, meshLen, increment, set); break;
            default : printf("[ERROR] CopyNode\n");
        }
    }
}

void print_BEAM(FILE *f, int elmIndex, int nodeIndex, int nodePp, int typb)
{
    fprintf(f, "BEAM :(%5d)(%5d:%5d) TYPB(%3d)  Y-NODE(     )\n", elmIndex, nodeIndex, nodeIndex + nodePp, typb);
}

void print_QUAD(FILE *f, int elmIndex, int startNode, const int node_pp[], int dir1, int dir2, int TYPQ)
{
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", elmIndex, startNode, startNode + node_pp[dir1], startNode + node_pp[dir1] + node_pp[dir2], startNode + node_pp[dir2], TYPQ);
}

void print_HEXA(FILE *f, int EleIndex, int Node_S, const int nodePp[], int TYPH)
{
    fprintf(f, "HEXA :(%5d)(%5d:%5d:%5d:%5d:%5d:%5d:%5d:%5d) TYPH(%3d)\n", EleIndex, Node_S, Node_S + nodePp[0], Node_S + nodePp[0] + nodePp[1], Node_S + nodePp[1], Node_S + nodePp[2], Node_S + nodePp[0] + nodePp[2], Node_S + nodePp[0] + nodePp[1] + nodePp[2], Node_S + nodePp[1] + nodePp[2], TYPH);
}

void print_LINE(FILE *f, int elmIndex, int nodeIndex1, int nodeIndex3, int pp)
{
    fprintf(f, "LINE :(%5d)(%5d:%5d:%5d:%5d) TYPL(  1)\n", elmIndex, nodeIndex1, nodeIndex1 + pp, nodeIndex3, nodeIndex3 + pp);
}

void print_FILM(FILE *f, int elmIndex, int face1, int face2, const int nodePp[], int dir1, int dir2, int typf)
{
    fprintf(f, "FILM :(%5d)(%5d:%5d:%5d:%5d:%5d:%5d:%5d:%5d) TYPF(%3d)\n", elmIndex, face1, face1 + nodePp[dir1], face1 + nodePp[dir1] + nodePp[dir2], face1 + nodePp[dir2], face2, face2 + nodePp[dir1], face2 + nodePp[dir1] + nodePp[dir2], face2 + nodePp[dir2], typf);
}

void print_join(FILE *f, int s1, int e1, int i1, int s2, int e2, int i2)
{
    fprintf(f, "JOIN :NODE  S(%5d)-E(%5d)-I(%5d)  WITH  S(%5d)-E(%5d)-I(%5d)\n", s1, e1, i1, s2, e2, i2);
}

void print_rest(FILE *f, int s, int e, int i, int rc, int inc, int set)
{
    fprintf(f, "REST :NODE  S(%5d)-E(%5d)-I(%5d)  RC=(%03d000) INC(%5d)-SET(%4d)\n", s, e, i, rc, inc, set);
}

void print_sub1(FILE *f, int s, int e, int i, int dir, int master, int mDir)
{
    fprintf(f, "SUB1 :NODE  S(%5d)-E(%5d)-I(%5d)-D(%1d)  M(%5d)-D(%1d)  F=1\n", s, e, i, dir, master, mDir);
}

void print_etyp(FILE *f, int s, int e, int i, int type, int inc, int set)
{
    fprintf(f, "ETYP :ELM  S(%5d)-E(%5d)-I(%5d)  TYPE(%3d)  INC(%5d)-SET(%4d)\n", s, e, i, type, inc, set);
}

/*COPYELMデータ書き込み*/
int print_COPYELM(FILE *f, int elm_S, int elm_E, int elm_Inter, int elm_Inc, int node_Inc, int set)
{
    char end[6] = " ";
    char interval[6] = " ";
    if (elm_E != 0)
    {
        sprintf(end, "%d", elm_E);
    }
    if (elm_Inter != 0)
    {
        sprintf(interval, "%d", elm_Inter);
    }
    fprintf(f, "COPY :ELM  S(%5d)-E(%5s)-I(%5s)   INC(%5d)-NINC(%5d)-SET(%4d)\n", elm_S, end, interval, elm_Inc, node_Inc, set);
    if(elm_E != 0)
    {
        return elm_E + elm_Inc * set;
    }else{
        return elm_S + elm_Inc * set;
    }
}

int plot_node(FILE *f, int startNode, const StartEnd *point, const Geometry geo[], const int nodePp[])
{
    int dir = 0;
    int _dir;
    int index = startNode;
    int delt = 0;
    double cordi[XYZ];
    //節点定義
    for(int i = 0; i < XYZ; i++)
    {
        cordi[i] = point_to_coordinate(point->start[i], geo[i].mesh.length);
    }
    print_NODE(f, index, cordi);
    //節点コピー
    for(dir = 0; dir < XYZ; dir++)
    {
        if(point->end[dir] > point->start[dir])
        {
            index = startNode;
            for(int i = point->start[dir], cnt = 0; i < point->end[dir]; i += cnt)
            {
                cnt = count_consecutive(i, point->end[dir], geo[dir].mesh.length);
                if(cnt < 0)
                {
                    printf("[ERROR] X\n");
                    return -1;
                }
                print_COPYNODE(f, index, 0, 0, geo[dir].mesh.length[i], nodePp[dir], cnt, dir);
                index += cnt * nodePp[dir];
            }
            delt = index - startNode;
            _dir = dir;
            dir++;
            break;
        }
    }
    for(; dir < XYZ; dir++)
    {
        if(point->end[dir] > point->start[dir])
        {
            index = startNode;
            for(int i = point->start[dir], cnt = 0; i < point->end[dir]; i += cnt)
            {
                cnt = count_consecutive(i, point->end[dir], geo[dir].mesh.length);
                if(cnt < 0)
                {
                    printf("[ERROR] Y\n");
                    return -1;
                }
                print_COPYNODE(f, index, index + delt, nodePp[_dir], geo[dir].mesh.length[i], nodePp[dir], cnt, dir);
                index += cnt * nodePp[dir];
            }
            dir++;
            break;
        }
    }
    if(dir < XYZ && point->end[dir] > point->start[dir])
    {
        for(int j = point->start[1]; j <= point->end[1]; j++)
        {
            index = startNode + (j - point->start[1]) * nodePp[1];
            for(int i = point->start[2], cnt = 0; i < point->end[2]; i += cnt)
            {
                cnt = count_consecutive(i, point->end[2], geo[2].mesh.length);
                if(cnt < 0)
                {
                    printf("[ERROR] Z\n");
                    return -1;
                }
                print_COPYNODE(f, index, index + delt, nodePp[0], geo[2].mesh.length[i], nodePp[2], cnt, 2);
                index += cnt * nodePp[2];
            }
        }
    }
    return startNode + (point->end[0] - point->start[0]) * nodePp[0] + (point->end[1] - point->start[1]) * nodePp[1] + (point->end[2] - point->start[2]) * nodePp[2];
}

void generate_beam(FILE *f, const Increment *pp, const StartEnd *point, const NodeElm *start, const Geometry geo[], int typb)
{
    int dir;
    for(int i = 0; i < XYZ; i++)
    {
        if(point->end[i] != point->start[i])
        {
            dir = i;
            break;
        }
    }
    //節点定義
    plot_node(f, start->node, point, geo, pp->node);
    //要素定義
    print_BEAM(f, start->elm, start->node, pp->node[dir], typb);
    print_COPYELM(f, start->elm, 0, 0, 1, pp->node[dir], point->end[dir] - point->start[dir] - 1);
}

void generate_quad(FILE *f, const Increment *pp, const StartEnd *point, const NodeElm *start, Geometry geo[], int typq)
{
    int dir1 = 0, dir2 = 1;
    for(int i = 0; i < 2; i++)
    {
        if(point->end[i] == point->start[i])
        {
            if(i < 1)
            {
                dir1++;
            }
            dir2++;
            break;
        }
    }
    //節点定義
    plot_node(f, start->node, point, geo, pp->node);
    //要素定義
    print_QUAD(f, start->elm, start->node, pp->node, dir1, dir2, typq);
    print_COPYELM(f, start->elm, 0, 0, pp->elm[dir1], pp->node[dir1], point->end[dir1] - point->start[dir1] - 1);
    print_COPYELM(f, start->elm, start->elm + (point->end[dir1] - point->start[dir1] - 1) * pp->elm[dir1], pp->elm[dir1], pp->elm[dir2], pp->node[dir2], point->end[dir2] - point->start[dir2] - 1);
}

struct nodeElm generate_hexa(FILE *f, const NodeElm *start, const Increment *pp, const StartEnd *point, const Geometry geo[], int typh)
{
    int delt;
    struct nodeElm index;
    struct nodeElm returnIndex;
    index.node = start->node;
    index.elm  = start->elm;
    //節点定義
    returnIndex.node = plot_node(f, start->node, point, geo, pp->node);
    fprintf(f, "\n");
    //要素定義
    print_HEXA(f, start->elm, start->node, pp->node, typh);
    //要素コピー
    //x
    returnIndex.elm = print_COPYELM(f, index.elm, 0, 0, pp->elm[0], pp->node[0], point->end[0] - point->start[0] - 1);
    //y
    delt = (point->end[0] - point->start[0] - 1) * pp->elm[0];
    returnIndex.elm = print_COPYELM(f, index.elm, index.elm + delt, pp->elm[0], pp->elm[1], pp->node[1], point->end[1] - point->start[1] - 1);
    //z
    for(int i = 0; i < point->end[1] - point->start[1]; i++)
    {
        returnIndex.elm = print_COPYELM(f, index.elm + pp->elm[1] * i, index.elm + delt + pp->elm[1] * i, pp->elm[0], pp->elm[2], pp->node[2], point->end[2] - point->start[2] - 1);
    }
    return returnIndex;
}

void generate_line(FILE *f, int elm, int hexaNode, int beamNode, int nodePp, int set)
{
    const int elmPPz = 1;
    int index = elm;
    print_LINE(f, index, hexaNode, beamNode, nodePp);
    print_COPYELM(f, elm, 0, 0, elmPPz, nodePp, set);
}

struct nodeElm add_reber(FILE *f, struct increment pp, struct modelsize rcs, struct geometry geo[], struct nodeElm startIndex)
{
    const int dir = 2;
    int typb = 1;
    int rebarNum, beamLineDelt, concreteStartNode;
    struct nodeElm index;
    struct startEnd point;
    index  = startIndex;
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs.rebar[i].cordi[0] < 0)
        {
            rebarNum = i;
            break;
        }
    }
    beamLineDelt = next_index((geo[2].boundary[4] - geo[2].boundary[1]) * rebarNum);
    fprintf(f, "\n----REBAR----");
    for(int j = 0; j < rebarNum; j++)
    {
        index.node = startIndex.node + j;
        int xIndex = coordinate_to_point(rcs.rebar[j].cordi[0], geo[0].mesh.length) - geo[0].boundary[2];
        int yIndex = coordinate_to_point(rcs.rebar[j].cordi[1], geo[1].mesh.length) - geo[1].boundary[0];
        if(coordinate_to_point(rcs.rebar[j].cordi[0], geo[0].mesh.length) > geo[0].boundary[3])
            xIndex++;
        int concreteAddNumber = pp.node[0] * xIndex + pp.node[1] * yIndex;

        fprintf(f, "\n----\n");
        for(int i = 0; i < 2; i++)
        {
            point.start[i] = coordinate_to_point(rcs.rebar[j].cordi[i], geo[i].mesh.length);
            point.end  [i] = coordinate_to_point(rcs.rebar[j].cordi[i], geo[i].mesh.length);
        }
        for(int i = 1; i < 4; i++)
        {
            point.start[2]   = geo[2].boundary[i];
            point.end  [2]   = geo[2].boundary[i + 1];
            generate_beam(f, &pp, &point, &index, geo, typb);
            concreteStartNode = (1 + (geo[2].boundary[i] + i) * pp.node[2]) + concreteAddNumber;
            generate_line(f, index.elm + beamLineDelt, concreteStartNode, index.node, pp.node[2], geo[dir].boundary[i + 1] - geo[dir].boundary[i] - 1);
            concreteStartNode += pp.node[dir]   * (geo[dir].boundary[i + 1] - geo[dir].boundary[i] + 1);
            index.node += pp.node[dir]   * (geo[dir].boundary[i + 1] - geo[dir].boundary[i] + 1);
            index.elm  += geo[dir].boundary[i + 1] - geo[dir].boundary[i];
        }
    }
    index.elm  += beamLineDelt;
    return index;
}

struct nodeElm add_concrete(FILE *f, struct nodeElm startIndex, struct increment pp, struct geometry geo[])
{
    int typh[5] = {5, 1, 3, 1, 5};
    struct nodeElm index;
    struct nodeElm returnIndex;
    struct startEnd point;
    index = startIndex;

    point.start[1] = geo[1].boundary[0];
    point.end  [1] = geo[1].boundary[2];
    for(int j = 0; j < 5; j++)
    {
        fprintf(f, "\n----TYPH %1d----", typh[j]);
        point.start[2] = geo[2].boundary[j];
        point.end  [2] = geo[2].boundary[j + 1];
        for(int i = 2; i < 4; i++)
        {
            fprintf(f, "\n----\n");
            point.start[0] = geo[0].boundary[i];
            point.end  [0] = geo[0].boundary[i + 1];
            returnIndex = generate_hexa(f, &index, &pp, &point, geo, typh[j]);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1) * pp.node[0];
            index.elm  += (geo[0].boundary[i + 1] - geo[0].boundary[i])     * pp.elm [0];
        }
        index.node = startIndex.node + (geo[2].boundary[j + 1] + j + 1) * pp.node[2];
        index.elm  = startIndex.elm  +  geo[2].boundary[j + 1]          * pp.elm [2];
    }
    return returnIndex;
}

NodeElm joint_quad(FILE *f, struct nodeElm startIndex, struct geometry geo[], struct increment pp, struct modelsize rcs, int jointStartNode)
{
    int film;
    int typq[5] = {0};
    NodeElm  index;
    StartEnd point;
    Increment jointPp = pp;
    jointPp.elm[0] = 1;
    jointPp.elm[1] = pp.elm[1] + 4;
    jointPp.elm[2] = jointPp.elm[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 2);

    int filmStart = startIndex.elm + (geo[0].boundary[4] - geo[0].boundary[2] + 3) * (geo[1].boundary[2] - geo[1].boundary[0] + 2) * (geo[2].boundary[3] - geo[2].boundary[2] + 2);
    filmStart = next_index(filmStart);

    printf("filmstart -> %d\n", filmStart);
    index = startIndex;
    const int concreteSteelDelt = startIndex.node - (jointStartNode + (geo[2].boundary[2] + 2) * pp.node[2]);
    fprintf(f, "\n----COLUMN QUAD----\n");
    //x直交面
    fprintf(f, "\n----x\n");
    set_start_end(&point, 0, 2, geo[1].boundary, 1);
    set_start_end(&point, 2, 3, geo[2].boundary, 2);
    index.elm  = startIndex.elm + jointPp.elm[1] + jointPp.elm[2];
    typq[2] = 9;
    typq[3] = 5;
    typq[4] = 13;
    for(int i = 2; i <= 4; i++)
    {
        index.node = startIndex.node + (geo[0].boundary[i] - geo[0].boundary[2]) * pp.node[0];
        index.elm  = startIndex.elm  + (geo[0].boundary[i] - geo[0].boundary[2] + i - 2) * jointPp.elm[0] + jointPp.elm[1] + jointPp.elm[2];
        set_start_end(&point, i, i, geo[0].boundary, 0);
        generate_quad(f, &jointPp, &point, &index, geo, typq[i]);
    }
    //film要素
    fprintf(f, "\n----\n");
    film = filmStart + jointPp.elm[1] + 2 * jointPp.elm[2];
    index = startIndex;
    int delt = concreteSteelDelt;
    for(int i = 0; i < 2; i++)
    {
        print_FILM(f, film, index.node - delt, index.node, pp.node, 2, 1, 1);
        print_COPYELM(f, film, 0, 0, jointPp.elm[1], pp.node[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        print_COPYELM(f, film, film + (geo[1].boundary[2] - geo[1].boundary[0] - 1) * jointPp.elm[1], jointPp.elm[1], jointPp.elm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        film  += geo[0].boundary[3] - geo[0].boundary[2] + 1;
        index.node += geo[0].boundary[3] - geo[0].boundary[2]; 
        print_FILM(f, film, index.node - delt, index.node, pp.node, 1, 2, 1);
        print_COPYELM(f, film, 0, 0, jointPp.elm[1], pp.node[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        print_COPYELM(f, film, film + (geo[1].boundary[2] - geo[1].boundary[0] - 1) * jointPp.elm[1], jointPp.elm[1], jointPp.elm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        film += jointPp.elm[0];
        delt -= pp.node[0];
    }

    //y直交面
    fprintf(f, "\n----y\n");
    index = startIndex;
    typq[0] = 11;
    typq[2] = 2;
    set_start_end(&point, 2, 3, geo[2].boundary, 2);
    for(int j = 0; j <= 2; j += 2)
    {
        for(int i = 2; i <= 3; i++)
        {
            index.node = startIndex.node + (geo[0].boundary[i] - geo[0].boundary[2] + 1) * pp.node[0] + (geo[1].boundary[j]) * pp.node[1];
            set_start_end(&point, i, i + 1, geo[0].boundary, 0);
            point.start[0]++;
            point.end[0]--;
            set_start_end(&point, j, j, geo[1].boundary, 1);
            plot_node(f, index.node, &point, geo, pp.node);
            index.elm  = startIndex.elm  + (geo[0].boundary[i] - geo[0].boundary[2] + 1 + i - 2) * jointPp.elm[0] + (geo[1].boundary[j] + j / 2) * jointPp.elm[1] + jointPp.elm[2];
            index.node -= pp.node[0];
            point.start[0]--;
            print_QUAD(f, index.elm, index.node, pp.node, 0, 2, typq[j]);
            print_COPYELM(f, index.elm, 0, 0, jointPp.elm[0], pp.node[0], geo[0].boundary[i + 1] - geo[0].boundary[i] - 1);
            print_COPYELM(f, index.elm, index.elm + (geo[0].boundary[i + 1] - geo[0].boundary[i] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        }
    }
    //film要素
    fprintf(f, "\n----\n");
    film = filmStart + jointPp.elm[0] + 2 * jointPp.elm[2];
    index = startIndex;
    delt = concreteSteelDelt;
    for(int i = 0; i < 2; i++)
    {
        print_FILM(f, film, index.node - delt, index.node, pp.node, 0, 2, 1);
        print_COPYELM(f, film, 0, 0, jointPp.elm[0], pp.node[0], geo[0].boundary[3] - geo[0].boundary[2] - 1);
        print_COPYELM(f, film, film + (geo[0].boundary[3] - geo[0].boundary[2] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        film  += (geo[1].boundary[2] - geo[1].boundary[0] + 1) * jointPp.elm[1];
        index.node += (geo[1].boundary[2] - geo[1].boundary[0]) * pp.node[1]; 
        print_FILM(f, film, index.node - delt, index.node, pp.node, 2, 0, 1);
        print_COPYELM(f, film, 0, 0, jointPp.elm[0], pp.node[0], geo[0].boundary[3] - geo[0].boundary[2] - 1);
        print_COPYELM(f, film, film + (geo[0].boundary[3] - geo[0].boundary[2] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        film  = filmStart + jointPp.elm[0] + jointPp.elm[2] + (geo[0].boundary[3] - geo[0].boundary[2] + 1) * jointPp.elm[0];
        index.node = startIndex.node + (geo[0].boundary[3] - geo[0].boundary[2]) * jointPp.elm[0];
        delt -= pp.node[0];
    }

    //z直交面
    fprintf(f, "\n----z\n");
    for(int k = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length); k < geo[1].boundary[2]; k++)
    {
        index.node = startIndex.node + k * pp.node[1];
        for(int j = 2; j < 4; j++)
        {
            for(int i = geo[0].boundary[j], cnt = 0; i < geo[0].boundary[j + 1] - 1; i += cnt)
            {
                cnt = count_consecutive(i, geo[0].boundary[j + 1] - 1, geo[0].mesh.length);
                if(cnt == -1)
                {
                    printf("[ERROR] quad_collumn\n");
                    index.elm  = -999;
                    index.node = -999;
                    return index;
                }
                print_COPYNODE(f, index.node, index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], geo[0].mesh.length[i], pp.node[0], cnt, 0);
                index.node += cnt * pp.node[0];
            }
            index.node ++;
        }
    }
    for(int j = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); j <= coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); j++)
    {
        if(j != geo[0].boundary[3])
        {
            index.node = startIndex.node + (j - geo[0].boundary[2]) * pp.node[0];
            for(int i = geo[1].boundary[0], cnt = 0; i < coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length) - 1; i += cnt)
            {
                cnt = count_consecutive(i, coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length) - 1, geo[1].mesh.length);
                print_COPYNODE(f, index.node, index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], geo[1].mesh.length[i], pp.node[1], cnt, 1);
                index.node += cnt * pp.node[1];
            }
        }
    }
    //四辺形要素
    for(int j = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length); j < geo[1].boundary[2]; j++)
    {
        index.node = startIndex.node + j * pp.node[1];
        index.elm  = startIndex.elm  + 1 * jointPp.elm[0] + (j + 1) * jointPp.elm[1];
        for(int i = 2; i < 4; i++)
        {
            print_QUAD(f, index.elm, index.node, pp.node, 0, 1, 4);
            print_QUAD(f, index.elm + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * jointPp.elm[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 3);
            print_COPYELM(f, index.elm, index.elm + (geo[2].boundary[3] -geo[2].boundary[2] + 1) * jointPp.elm[2], (geo[2].boundary[3] - geo[2].boundary[2] + 1) * jointPp.elm[2], jointPp.elm[0], pp.node[0], geo[0].boundary[i + 1] - geo[0].boundary[i] - 1);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i])     * pp.node[0];
            index.elm  += (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1) * jointPp.elm[0]; 
        }
    }
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); i++)
    {

        index.node = startIndex.node + (i - geo[0].boundary[2]) * pp.node[0];
        index.elm  = startIndex.elm  + (i - geo[0].boundary[2] + 1) * jointPp.elm[0] + 1 * jointPp.elm[1];
        if(i >= geo[0].boundary[3])
        {
            index.elm++;
        }
        print_QUAD(f, index.elm, index.node, pp.node, 0, 1, 7);
        print_QUAD(f, index.elm + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * jointPp.elm[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 6);
        print_COPYELM(f, index.elm, index.elm + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * jointPp.elm[2], (geo[2].boundary[3] - geo[2].boundary[2] + 1) * jointPp.elm[2], jointPp.elm[1], pp.node[1], coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length) - 1);
    }
    //film要素
    int flangeNum = geo[1].boundary[2] - geo[1].boundary[1];
    fprintf(f, "\n----\n");
    index = startIndex;
    delt = concreteSteelDelt;
    for(int i = 2; i <= 3; i++)
    {
        index.node = startIndex.node + (geo[0].boundary[i] - geo[0].boundary[2]) * pp.node[0] + geo[1].boundary[1] * pp.node[1];
        film  = filmStart + (geo[0].boundary[i] - geo[0].boundary[2] + 1) * jointPp.elm[0] + (geo[1].boundary[1] + 1) * jointPp.elm[1];
        if(i >= 3)
        {
            film  += jointPp.elm[0];
            delt -= pp.node[0];
        }
        print_FILM(f, film, index.node - delt - pp.node[2], index.node, pp.node, 1, 0, 1);
        print_FILM(f, film + jointPp.elm[2], index.node - delt, index.node, pp.node, 0, 1, 1);
        print_COPYELM(f, film, film + jointPp.elm[2], jointPp.elm[2], jointPp.elm[0], pp.node[0], geo[0].boundary[i + 1] - geo[0].boundary[i] - 1);
        if(flangeNum > 1)
        {
            print_COPYELM(f, film, film + (geo[0].boundary[i + 1] - geo[0].boundary[i] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[1], pp.node[1], flangeNum - 1);
            print_COPYELM(f, film + jointPp.elm[2], film + jointPp.elm[2] + (geo[0].boundary[i + 1] - geo[0].boundary[i] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[1], pp.node[1], flangeNum - 1);
        }
        print_FILM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 1);
        print_FILM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 1, 0, 1);
        print_COPYELM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2], film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2], jointPp.elm[2], jointPp.elm[0], pp.node[0], geo[0].boundary[i + 1] - geo[0].boundary[i] - 1);
        if(flangeNum > 1)
        {
            print_COPYELM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2], film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2] + (geo[0].boundary[i + 1] - geo[0].boundary[i] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[1], pp.node[1], flangeNum - 1);
            print_COPYELM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2], film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2] + (geo[0].boundary[i + 1] - geo[0].boundary[i] - 1) * jointPp.elm[0], jointPp.elm[0], jointPp.elm[1], pp.node[1], flangeNum - 1);
        }
    }
    index = startIndex;
    delt = concreteSteelDelt;
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); i++)
    {
        index.node = startIndex.node + (i - geo[0].boundary[2]) * pp.node[0];
        film = filmStart + (i - geo[0].boundary[2] + 1) * jointPp.elm[0] + jointPp.elm[1];
        if(i >= geo[0].boundary[3])
        {
            delt = concreteSteelDelt - pp.node[0];
        }
        print_FILM(f, film, index.node - delt - pp.node[2], index.node, pp.node, 0, 1, 1);
        print_FILM(f, film + jointPp.elm[2], index.node - delt, index.node, pp.node, 1, 0, 1);
        print_COPYELM(f, film, film + jointPp.elm[2], jointPp.elm[2], jointPp.elm[1], pp.node[1], geo[1].boundary[1] - 1);

        print_FILM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 1);
        print_FILM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 1, 0, 1);
        print_COPYELM(f, film + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * jointPp.elm[2], film + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * jointPp.elm[2], jointPp.elm[2], jointPp.elm[1], pp.node[1], geo[1].boundary[1] - 1);
    }
    
    //要素タイプ定義
    typq[0] = 8;
    typq[1] = 12;
    int number = geo[1].boundary[2] - geo[1].boundary[1];
    index.elm = startIndex.elm + (geo[1].boundary[1] + 1) * jointPp.elm[1] + jointPp.elm[2];
    for(int j = 0; j < 2; j++)
    {
        if(number == 1)
        {
            print_etyp(f, index.elm, 0, 0, typq[j], jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        }
        else if(number > 1)
        {
            print_etyp(f, index.elm, index.elm + (number - 1) * jointPp.elm[1], jointPp.elm[1], typq[j], jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        }
        index.elm += (geo[0].boundary[4] - geo[0].boundary[2] + 2) * jointPp.elm[0];
    }
    number = geo[0].boundary[3] - coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length);
    index.elm = startIndex.elm + (coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length) - geo[0].boundary[2] + 1) * jointPp.elm[0] + jointPp.elm[2];
    for(int j = 0; j < 2; j++)
    {
        if(number == 1)
        {
            print_etyp(f, index.elm, 0, 0, 10, jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        }
        else if(number > 1)
        {
            print_etyp(f, index.elm, index.elm + (number - 1) * jointPp.elm[0], jointPp.elm[0], 10, jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        }
        index.elm += (number + 1) * jointPp.elm[0];
    }
    index.node  = startIndex.node;
    index.node += (geo[0].boundary[4] - geo[0].boundary[2] + 1) * (geo[1].boundary[2] - geo[1].boundary[0] + 1) * (geo[2].boundary[3] - geo[2].boundary[2] + 1);
    index.elm   = filmStart;
    index.elm  += (geo[0].boundary[4] - geo[0].boundary[2] + 4) * (geo[1].boundary[2] - geo[1].boundary[0] + 2) * (geo[2].boundary[3] - geo[2].boundary[2] + 4);
    return index;
}

void hexa_beam(FILE *f, struct nodeElm startIndex, struct increment pp, struct geometry geo[])
{
    const int typh = 5;
    struct nodeElm  index;
    struct startEnd point;
    point.start[1] = geo[1].boundary[1];
    point.end  [1] = geo[1].boundary[2];
    point.start[2] = geo[2].boundary[2];
    point.end  [2] = geo[2].boundary[3];
    
    fprintf(f, "\n----HEXA BEAM----\n");
    for(int i = 0; i < 6; i += 5)
    {
        fprintf(f, "\n----\n");
        index.node = startIndex.node + geo[0].boundary[i] * pp.node[0];
        index.elm  = startIndex.elm  + geo[0].boundary[i] * pp.node[0] + pp.elm[2];
        point.start[0] = geo[0].boundary[i];
        point.end  [0] = geo[0].boundary[i + 1];
        generate_hexa(f, &index, &pp, &point, geo, typh);
    }
}

void quad_beam(FILE *f, struct nodeElm startIndex, struct increment ppColumn, struct increment ppBeam, struct geometry geo[], int jointStartNode)
{
    int flangeNum = geo[1].boundary[2] - geo[1].boundary[1];
    const int typq[4] = {0, 0, 4, 3};
    int jointNode = jointStartNode;
    int quadDelt, jointDelt, elmDelt;
    struct nodeElm  index;
    struct startEnd point;
    point.end  [1] = geo[1].boundary[2];

    fprintf(f, "\n----QUAD BEAM----\n");
    //節点定義
    for(int j = 1; j <= 4; j += 3) //x方向
    {
        point.start[0] = geo[0].boundary[j] + 1;
        point.end  [0] = geo[0].boundary[j + 1] - 1;
        for(int i = 2; i <= 3; i++) //z方向
        {
            point.start[1] = geo[1].boundary[1];
            point.start[2] = geo[2].boundary[i];
            point.end  [2] = geo[2].boundary[i];
            index.node = startIndex.node + (geo[0].boundary[j] + 1) * ppBeam.node[0] + (geo[2].boundary[i] - geo[2].boundary[2]) * ppBeam.node[2];
            plot_node(f, index.node, &point, geo, ppBeam.node);
        }
        point.start[1] = geo[1].boundary[2];
        point.start[2] = geo[2].boundary[2] + 1;
        point.end  [2] = geo[2].boundary[3] - 1;
        index.node = startIndex.node + (geo[0].boundary[j] + 1) * ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1] + ppBeam.node[2];
        plot_node(f, index.node, &point, geo, ppBeam.node);
    }
    //要素定義
    //接合部分
    quadDelt    = (geo[0].boundary[4] - geo[0].boundary[2] + 2) * ppBeam.node[0];
    jointDelt   = (geo[0].boundary[4] - geo[0].boundary[2])     * ppColumn.node[0];
    elmDelt     = (geo[0].boundary[4] - geo[0].boundary[2] + 1) * ppBeam.elm[0];

    index.node  = startIndex.node + (geo[0].boundary[2] - 1) * ppBeam.node[0];
    index.elm   = startIndex.elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0];
    for(int i = 0; i < flangeNum; i++)
    {
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + i * ppBeam.elm[1], index.node + i * ppBeam.node[1], jointNode + geo[1].boundary[1] * ppColumn.node[1] + i * ppColumn.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + i * ppColumn.node[1], index.node + ppBeam.node[1] + i * ppBeam.node[1], 4); 
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt + i * ppBeam.elm[1], jointNode + geo[1].boundary[1] * ppColumn.node[1] + jointDelt + i * ppColumn.node[1], index.node + quadDelt + i * ppBeam.node[1], index.node + ppBeam.node[1] + quadDelt + i * ppBeam.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + jointDelt + i * ppColumn.node[1], 4); 
    }
    index.node += (geo[2].boundary[3] - geo[2].boundary[2]) * ppBeam.node[2];
    index.elm  += (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppBeam.elm[2];
    jointNode  += (geo[2].boundary[3] - geo[2].boundary[2]) * ppColumn.node[2];
    for(int i = 0; i < flangeNum; i++)
    {
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + i * ppBeam.elm[1], index.node + i * ppBeam.node[1], jointNode + geo[1].boundary[1] * ppColumn.node[1] + i * ppColumn.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + i * ppColumn.node[1], index.node + ppBeam.node[1] + i * ppBeam.node[1], 3); 
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt + i * ppBeam.elm[1], jointNode + geo[1].boundary[1] * ppColumn.node[1] + jointDelt + i * ppColumn.node[1], index.node + quadDelt + i * ppBeam.node[1], index.node + ppBeam.node[1] + quadDelt + i * ppBeam.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + jointDelt + i * ppColumn.node[1], 3); 
    }

    jointNode  = jointStartNode;
    index.node = startIndex.node + (geo[0].boundary[2] - 1) * ppBeam.node[0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = startIndex.elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm [1] + ppBeam.elm [2];
    for(int i = 0; i < geo[2].boundary[3] - geo[2].boundary[2]; i++)
    {
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm, index.node, jointNode + geo[1].boundary[2] * ppColumn.node[1], jointNode + geo[1].boundary[2] * ppColumn.node[1] + ppColumn.node[2], index.node + ppBeam.node[2], 1); 
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt, jointNode + geo[1].boundary[2] * ppColumn.node[1] + jointDelt, index.node + quadDelt, index.node + ppBeam.node[2] + quadDelt, jointNode + geo[1].boundary[2] * ppColumn.node[1] + ppColumn.node[2] + jointDelt, 1); 
    jointNode  += ppColumn.node[2];
    index.node += ppBeam.node[2];
    index.elm  += ppBeam.elm[2];
    }
    //梁
    quadDelt   = (geo[0].boundary[4] - geo[0].boundary[1] + 1) * ppBeam.node[0];
    elmDelt    = (geo[0].boundary[4] - geo[0].boundary[1] + 1) * ppBeam.elm[0];
    for(int i = 2; i <= 3; i++)
    {
        index.node = startIndex.node + ppBeam.node[0] + (geo[2].boundary[i] - geo[2].boundary[2]) * ppBeam.node[2];
        index.elm  = startIndex.elm  + ppBeam.elm [0] + (geo[2].boundary[i] - geo[2].boundary[2] + i - 2) * ppBeam.elm[2];
        print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 1, typq[i]);
        print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 1, typq[i]);    
        print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
        if(flangeNum > 1)
        {
            print_COPYELM(f, index.elm, index.elm + (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0], ppBeam.elm[0], ppBeam.elm[1], ppBeam.node[1], flangeNum - 1);
            print_COPYELM(f, index.elm + elmDelt, index.elm + elmDelt + (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0], ppBeam.elm[0], ppBeam.elm[1], ppBeam.node[1], flangeNum - 1);
        }
    }
    index.node = startIndex.node + ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = startIndex.elm  + ppBeam.elm [0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 2, 1);
    print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 2, 1);    
    print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
    
    index.elm = startIndex.elm + ppBeam.elm[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    elmDelt = (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    index.elm += (geo[0].boundary[4] - geo[0].boundary[1] + 1) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
}

void joint_nodes(FILE *f, int startNode, struct geometry geo[], struct modelsize rcs, int pp[])
{
    int node;
    //x直交面
    fprintf(f, "\n----\n");
    int delt = (geo[2].boundary[2] - geo[2].boundary[0] + 1) * pp[2];
    for(int i = 0; i <= geo[1].boundary[2]; i++)
    {
        node  = startNode + (geo[0].boundary[3] - geo[0].boundary[2]) * pp[0] + i * pp[1];
        print_join(f, node, node + delt, pp[2], node + pp[0], node + delt + pp[0], pp[2]);
        node += (geo[2].boundary[3] + 3) * pp[2];
        print_join(f, node, node + delt, pp[2], node + pp[0], node + delt + pp[0], pp[2]);
    }
    //z直交面
    delt = (geo[1].boundary[2] - geo[1].boundary[0]) * pp[1];
    for(int i = geo[0].boundary[2]; i <= geo[0].boundary[4]; i++)
    {
        node = startNode + (i - geo[0].boundary[2]) * pp[0] + geo[2].boundary[1] * pp[2];
        if(i > geo[0].boundary[3])
        {
            node += pp[0];
        }
        print_join(f, node, node + delt, pp[1], node + pp[2], node + delt + pp[2], pp[1]);
        node += (geo[2].boundary[4] - geo[2].boundary[1] + 3) * pp[2];
        print_join(f, node, node + delt, pp[1], node + pp[2], node + delt + pp[2], pp[1]);
    }
    //接合部
    delt = geo[1].boundary[1] * pp[1];
    for(int i = geo[0].boundary[2]; i <= geo[0].boundary[4]; i++)
    {
        if(i <= coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length) || i >= coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length))
        {
            node = startNode + (i - geo[0].boundary[2]) * pp[0] + (geo[2].boundary[2] + 1) * pp[2];
            if(i > geo[0].boundary[3])
            {
                node += pp[0];
            }
            print_join(f, node, node + delt, pp[1], node + pp[2], node + delt + pp[2], pp[1]);
            node += (geo[2].boundary[3] - geo[2].boundary[2] + 1) * pp[2];
            print_join(f, node, node + delt, pp[1], node + pp[2], node + delt + pp[2], pp[1]);
        }
    }
}

void pin(FILE *f, int startNode, struct geometry geo[], int pp[])
{
    fprintf(f, "\n----\n");
    int node = startNode;
    int delt = (geo[1].boundary[2] - geo[1].boundary[1]) * pp[1];
    print_rest(f, node, node + delt, pp[1], 1, pp[2], geo[2].boundary[3] - geo[2].boundary[2]);
    node += geo[0].boundary[6] * pp[0];
    print_rest(f, node, node + delt, pp[1], 1, pp[2], geo[2].boundary[3] - geo[2].boundary[2]);
}

void pin_roller(FILE *f, int startNode, struct loadNode load, struct geometry geo[], int pp[])
{
    int node = startNode;
    int delt = (geo[1].boundary[2] - geo[1].boundary[0]) * pp[1];
    for(int i = geo[0].boundary[2]; i <= geo[0].boundary[4]; i++)
    {
        node = startNode + (i - geo[0].boundary[2]) * pp[0];
        if(i > geo[0].boundary[3])
        {
            node += pp[0];
        }
        if(i != geo[0].boundary[3])
        {
            print_sub1(f, node, node + delt, pp[1], 1, load.node1, 1);
        }
        else
        {
            print_sub1(f, node, node + delt - pp[1], pp[1], 1, load.node1, 1);
        }
    }
    for(int i = geo[0].boundary[2]; i <= geo[0].boundary[4]; i++)
    {
        node = startNode + (i - geo[0].boundary[2]) * pp[0] + (geo[2].boundary[5] + 4) * pp[2];
        if(i > geo[0].boundary[3])
        {
            node += pp[0];
        }
        if(i != geo[0].boundary[3])
        {
            print_sub1(f, node, node + delt, pp[1], 1, load.node2, 1);
        }
        else
        {
            print_sub1(f, node, node + delt - pp[1], pp[1], 1, load.node2, 1);
        }
    }
}

void cut_surface(FILE *f, struct geometry geo[], int columnStart, int jointStart, int beamStart, int columnPp[], int beamPp[])
{
    int column = columnStart + geo[1].boundary[2] * columnPp[1];
    int beam   = beamStart   + (geo[1].boundary[2] - geo[1].boundary[1]) * beamPp[1];
    //柱
    print_rest(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] + 1) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[2] - geo[2].boundary[0] + 2);
    column = columnStart + geo[1].boundary[2] * columnPp[1] + (geo[2].boundary[3] + 2) * columnPp[2];
    print_rest(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] + 1) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[5] - geo[2].boundary[3] + 2);
    print_rest(f, jointStart + geo[1].boundary[2] * columnPp[1], jointStart + geo[1].boundary[2] * columnPp[1] + (geo[0].boundary[4] - geo[0].boundary[2]) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
    //梁
    print_rest(f, beam, beam + (geo[0].boundary[2] - 1) * beamPp[0], beamPp[0], 10, beamPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
    beam += (geo[0].boundary[4] + 1) * beamPp[0];
    print_rest(f, beam, beam + (geo[0].boundary[6] - geo[0].boundary[4] - 1) * beamPp[0], beamPp[0], 10, beamPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
}

void add_typh(FILE *f, struct geometry geo[], struct modelsize rcs, int columnStart, int columnPp[])
{
    int column;
    int max[2];
    int min[2];
    double maxCordi[2] = {rcs.rebar[0].cordi[0], rcs.rebar[0].cordi[1]};
    double minCordi[2] = {rcs.rebar[0].cordi[0], rcs.rebar[0].cordi[1]};
    for(int j = 0; j < 2; j++)
    {
        for(int i = 1; rcs.rebar[i].cordi[j] > 0; i++)
        {
            double cordi = rcs.rebar[i].cordi[j];
            if(cordi < minCordi[j])
            {
                minCordi[j] = cordi;
                continue;
            }
            else if(cordi > maxCordi[j])
            {
                maxCordi[j] = cordi;
                continue;
            }
        }
    }
    max[0] = coordinate_to_point(maxCordi[0], geo[0].mesh.length) - geo[0].boundary[2];
    min[0] = coordinate_to_point(minCordi[0], geo[0].mesh.length) - geo[0].boundary[2];
    max[1] = coordinate_to_point(maxCordi[1], geo[1].mesh.length);
    min[1] = coordinate_to_point(minCordi[1], geo[1].mesh.length);
    //加力治具
    for(int i = 0; i <= 4; i += 4)
    {
        column = columnStart + geo[2].boundary[i] * columnPp[2];
        print_etyp(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 5, columnPp[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
    }
    //かぶり
    for(int j = 1; j <= 3; j += 2)
    {
        for(int i = 0; i < min[1]; i++)
        {
            column = columnStart + i * columnPp[1] + geo[2].boundary[j] * columnPp[2];
            print_etyp(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 2, columnPp[2], geo[2].boundary[j + 1] - geo[2].boundary[j] - 1);
        }
        for(int i = 0; i < geo[0].boundary[4] - geo[0].boundary[2]; i++)
        {
            if(i < min[0] || i >= max[0])
            {
                column = columnStart + i * columnPp[0] + min[1] * columnPp[1] + geo[2].boundary[j] * columnPp[2];
                print_etyp(f, column, column + (geo[1].boundary[2] - min[1] - 1) * columnPp[1], columnPp[1], 2, columnPp[2], geo[2].boundary[j + 1] - geo[2].boundary[j] - 1);
            }
        }
    }
    //接合部外部
    max[0] = coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length) - geo[0].boundary[2];
    min[0] = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length) - geo[0].boundary[2];
    //max[1] = coordinate_to_point((rcs.column.width + rcs.beam.width) / 2, geo[1].mesh.length);
    min[1] = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length);
    for(int i = 0; i < min[1]; i++)
    {
        column = columnStart + i * columnPp[1] + geo[2].boundary[2] * columnPp[2];
        print_etyp(f, column, column + (min[0] - 1) * columnPp[0], columnPp[0], 4, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        column += max[0] * columnPp[0];
        print_etyp(f, column, column + (geo[0].boundary[4] - max[0] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 4, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    }
}

void modeling_rcs(const char *inputFileName, int opt1)
{
    /*todo
        ---要素タイプ---
        film要素

        データチェック

        .logファイル

        quad_beamきたない
    */

    /*名称
        input.txt    : 入力ファイル
        out.ffi      : 出力ファイル
        mesh         : 分割情報
        0, 1, 2      : x, y, z
        point        : 格子点
        pp           : ++ （インクリメント）
   */

    /*メモ
        格子の原点は(0, 0, 0)
        主筋入力データの座標は柱左下を原点
        コマンドライン引数でフルの選択
        入力ファイル名の指定
        変数名　キャメルケース
        関数名　スネークケース
    */

    const int startNode = 1;
    const int startElm  = 1;
    ModelSize rcs;
    Geometry  modelGeometry[XYZ];
    Increment columnPp;
    Increment beamPp;
    NodeElm   index;
    LoadNode  loadNode;

    printf("\n[START]\n");

    if(load_inputdata(inputFileName, modelGeometry, &rcs) == 1)
    {
        return ;
    }

    //データチェック
    printf("\n------------------[ DATE CHECK ]------------------\n\n");
    printf("non\n----\n");

    //x, y, z方向は入力範囲が半分
    //yはハーフモデルのためそのまま
    symmetrical_input(&modelGeometry[0]);
    symmetrical_input(&modelGeometry[2]);

    get_boundary(rcs, modelGeometry);
    
    //試験体情報表示
    console_model_sizes(rcs);

    //メッシュ情報表示
    console_mesh(modelGeometry);
    printf("\n");

    //境界線情報表示
    console_boundary(modelGeometry);

    //インクリメント
    get_increment(&columnPp, &beamPp, modelGeometry);
    console_increment(columnPp);
    console_increment(beamPp);

    get_load_node(&loadNode, startNode, columnPp.node, modelGeometry);

    //ファイルオープン
    FILE *fout = fopen(OUT_FILE_NAME,"w");
    if(fout == NULL)
    {
        printf("[ERROR] out.ffi cant open.\n");
        return ;
    }

    //雛形
    print_head_template(fout, loadNode);    
    printf("\n------------------[ COLUMN ]------------------\n");

    //六面体要素
    index.node = startNode;
    index.elm  = startElm;
    console_index(index, "concrete start");
    index = add_concrete(fout, index, columnPp, modelGeometry);

    //線材要素
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    console_index(index, "rebar start");
    index = add_reber(fout, columnPp, rcs, modelGeometry, index);
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    int jointStartNode = index.node;
    printf("joint -> %d\n", jointStartNode);

    printf("\n------------------[ BEAM ]------------------\n");
    //四辺形要素
    console_index(index, "quad column start");
    index = joint_quad(fout, index, modelGeometry, columnPp, rcs, startNode);
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    int beamStartNode = index.node;

    //六面体要素
    console_index(index, "hexa beam");
    hexa_beam(fout, index, beamPp, modelGeometry);

    //四辺形要素
    quad_beam(fout, index, columnPp, beamPp, modelGeometry, jointStartNode);
    
    //
    joint_nodes(fout, startNode, modelGeometry, rcs, columnPp.node);

    if(opt1 == 'c')
    {
        pin(fout, beamStartNode, modelGeometry, beamPp.node);
        pin_roller(fout, startNode, loadNode, modelGeometry, columnPp.node);
    }else if(opt1 == 'b')
    {
        pin(fout, beamStartNode, modelGeometry, beamPp.node);
        pin_roller(fout, startNode, loadNode, modelGeometry, columnPp.node);
    }
    
    cut_surface(fout, modelGeometry, startNode, jointStartNode, beamStartNode, columnPp.node, beamPp.node);
    //六面体要素番号
    add_typh(fout, modelGeometry, rcs, startElm, columnPp.elm);


    print_tail_template(fout, loadNode, startElm, modelGeometry, rcs.column.Fc, columnPp.elm);
    fclose(fout);

    printf("\n[END]\n");

}

void show_usage()
{
    printf(
        "options\n"
        "-b : beam -> roller  column -> pin\n"
        "-f : full model \n"
    );
}

int main(int argc, char *argv[])
{
    int         optchar;
    int         opt1 = 'c';
    const char* inputFile = NULL;

    if(argc < 2)
    {
        show_usage();
        return 1;
    }
    while((optchar = getopt(argc, argv, OPTCHAR)) != -1)
    {
        switch(optchar){
            case 'b': //beam 梁加力
                opt1 = 'b';
                break;
            default:
                show_usage();
                return 1;
        }
    }
    if(optind < argc){
        inputFile = argv[optind];
    }else{
        fprintf(stderr, "No input file specified\n");
        return 1;
    }
    if(inputFile != NULL){
        printf("Input file: %s\n", inputFile);
        modeling_rcs(inputFile, opt1);
    }
    return 0;
}