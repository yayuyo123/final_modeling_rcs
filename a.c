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

void read_csv(FILE *f, Mesh *mesh)
{
    int cnt = 0;
    if(fgetc(f) == '(')
    {
        fscanf(f, "%lf[^,)]", &mesh->length[cnt++]);
        while (fgetc(f) != ')')
        {
            fscanf(f, "%lf[^,)]", &mesh->length[cnt++]);
        }
        mesh->number = cnt;
        mesh->length[cnt] = -999.0;
    }
}


ModelSize load_size(FILE *f)
{
    ModelSize modelSize;    
    //柱梁
    fscanf(f, "Column : Span(%lf) Width(%lf) Depth(%lf) X_Center(%lf) Y_Center(%lf) Fc(%lf)",
        &modelSize.column.span, &modelSize.column.width, &modelSize.column.depth, &modelSize.column.center[0], &modelSize.column.center[1], &modelSize.column.Fc);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "Beam   : Span(%lf) Width(%lf) Depth(%lf) Y_Center(%lf) Z_Center(%lf)",
        &modelSize.beam.span, &modelSize.beam.width, &modelSize.beam.depth, &modelSize.beam.center[1], &modelSize.beam.center[2]);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "xBeam  : Width(%lf)", &modelSize.xbeam.width);
    while((fgetc(f)) != '\n'){}
    //主筋
    for(int i = 0; i <= REBAR_MAX; i++)
    {
        if(fgetc(f) == 'R'){
            int index;
            fscanf(f, "ebar %d:", &index);
            if(index != i + 1){
                printf("[ERROR] index = %d i + 1 = %d\n", index, i + 1);
            }
            int result = fscanf(f, " X(%lf) Y(%lf)", &modelSize.rebar[i].cordi[0], &modelSize.rebar[i].cordi[1]);
            if (result != 2) { //値の読み込みが失敗した場合はループを終了
                printf("[ERROR]\n");
                break;
            }
            modelSize.rebar[i].cordi[0] += (modelSize.beam.span - modelSize.column.depth) / 2;
            while((fgetc(f)) != '\n'){}
        }else{
            modelSize.rebar[i].cordi[0] = -999.0;
            modelSize.rebar[i].cordi[1] = -999.0;
            while((fgetc(f)) != '\n'){}
            break;
        }
    }
    return modelSize;
}

/*mesh範囲を全体に広げる*/
void symmetrical_input(Mesh *mesh)
{
    int original_number = mesh->number;
    for (int i = 0; i < original_number; i++) {
        mesh->length[2 * original_number - i - 1] = mesh->length[i];
    }
    mesh->number *= 2;
}

Mesh load_mesh(FILE *f, int dir)
{
    Mesh mesh;
    const char str[3][16] = {"X_Mesh_Sizes", "Y_Mesh_Sizes", "Z_Mesh_Sizes"};
    for(int i = 0; str[dir][i] != '\0'; i++){
        int ch = fgetc(f);
        if(ch != str[dir][i]){
            fprintf(stderr, "[ERROR] dir = %d i = %d str = %c\n", dir, i, ch);
            break;
        }
    }
    read_csv(f, &mesh);
    while((fgetc(f)) != '\n'){}
    if(dir != 1){
        symmetrical_input(&mesh);
    }
    return mesh;
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

void get_boundary(int boundary[], const Mesh *mesh, int dir, const ModelSize *rcs)
{
    if(dir == 0){      //x
        boundary[0] = 0;
        boundary[1] = 1;
        boundary[2] = coordinate_to_point((rcs->beam.span - rcs->column.depth) / 2, mesh->length);
        boundary[3] = mesh->number / 2;
        boundary[4] = coordinate_to_point((rcs->beam.span + rcs->column.depth) / 2, mesh->length);
        boundary[5] = mesh->number - 1;
        boundary[6] = mesh->number;
    }else if(dir == 1){ //y
        boundary[0] = 0;
        boundary[1] = coordinate_to_point((rcs->column.width - rcs->beam.width) / 2, mesh->length);
        boundary[2] = mesh->number;
    }else if(dir == 2){ //z
        boundary[0] = 0;
        boundary[1] = 1;
        boundary[2] = coordinate_to_point((rcs->column.span - rcs->beam.depth) / 2, mesh->length);
        boundary[3] = coordinate_to_point((rcs->column.span + rcs->beam.depth) / 2, mesh->length);
        boundary[4] = mesh->number - 1;
        boundary[5] = mesh->number;
    }
}

Geometry set_geometry(FILE *f, int dir, const ModelSize *rcs)
{
    Geometry geo;
    geo.mesh = load_mesh(f, dir);
    get_boundary(geo.boundary, &geo.mesh, dir, rcs);
    return geo;
}

void get_load_node(LoadNode *pin, LoadNode *roller, int column, int beam, const int columnPp[], const int beamPp[], const Geometry geo[], int columnSpan, int opt1)
{
    LoadNode node;
    //柱
    node.node1 = column + (geo[0].boundary[3] - geo[0].boundary[2]) * columnPp[0] + geo[1].boundary[2] * columnPp[1];
    node.node2 = node.node1 + (geo[2].boundary[5] + 4) * columnPp[2];
    if(opt1 == 0)
    {
        roller->node1 = node.node1;
        roller->node2 = node.node2;
    }
    else if(opt1 == 1)
    {
        pin->node1 = node.node1;
        pin->node2 = node.node2;
    }
    //梁
    node.node1 = beam + (geo[1].boundary[2] - geo[1].boundary[1]) * beamPp[1] + (coordinate_to_point(columnSpan / 2, geo[2].mesh.length) - geo[2].boundary[2]) * beamPp[2];
    node.node2 = node.node1 + geo[0].boundary[6] * beamPp[0];
    if(opt1 == 0)
    {
        pin->node1 = node.node1;
        pin->node2 = node.node2;
    }
    else if(opt1 == 1)
    {
        roller->node1 = node.node1;
        roller->node2 = node.node2;
    }
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

int next_index(int lastNode) //1000の位を更新
{
    lastNode /= 1000;
    lastNode = (lastNode + 1) * 1000 + 1;
    return lastNode;
}

void set_start_end(struct startEnd *point, int start, int end, const int boundary[], int dir)
{
    point->start[dir] = boundary[start];
    point->end  [dir] = boundary[end];
}

void console_model_sizes(const ModelSize rcs)
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

void console_mesh(const Geometry geo[])
{
    int dir[3] = {'x', 'y', 'z'};
    printf("\n------------------[ MESH ]------------------\n");
    for(int j = 0; j < 3; j++)
    {
        printf("\n%c mesh count --> %d\n", dir[j], geo[j].mesh.number);
        printf("  i :");
        for(int i = 0; i < geo[j].mesh.number; i++)
        {
            printf(" %6d ", i);
        }
        printf("\nlen :");
        for(int i = 0; i < geo[j].mesh.number; i++)
        {
            printf(" %6.2f ", geo[j].mesh.length[i]);
        }
    }
}

void console_boundary(const Geometry geo[])
{
    printf("\n------------------[ BOUNDARY ]------------------\n");
    printf("X  : ");
    for(int i = 0; i <= 6; i++)
    {
        printf(" %4d ", geo[0].boundary[i]);
    }
    printf("\nY  : ");
    for(int i = 0; i <= 2; i++)
    {
        printf(" %4d ", geo[1].boundary[i]);
    }
    printf("\nZ  : ");
    for(int i = 0; i <= 5; i++)
    {
        printf(" %4d ", geo[2].boundary[i]);
    }
    printf("\n");
}

void console_increment(const Increment *pp)
{
    printf("\n------------------[ INCREMENT ]------------------\n");
    printf("         X     Y     Z\n");
    printf("node %5d %5d %5d\n", pp->node[0], pp->node[1], pp->node[2]);
    printf("elm  %5d %5d %5d\n", pp->elm[0], pp->elm[1], pp->elm[2]);
}

void console_index(struct nodeElm index, char *str)
{
    printf("\n------------------[ INDEX ]------------------\n");
    printf("%s\n", str);
    printf("node : %5d\n", index.node);
    printf("elm  : %5d\n", index.elm);
} 

//頭のひな形
void print_head_template(FILE *f, struct loadNode load, int opt1)
{
    int node = load.node1;
    if(opt1 == 0)
    {
        node = load.node2;
    }
    int dir = 1 + 2 * opt1;
    fprintf(f,
    "-------------------< FINAL version 11  Input data >---------------------\n"
    "TITL :\n"
    "EXEC :STEP (    1)-->(   11)  ELASTIC=( ) CHECK=(1) POST=(1) RESTART=( )\n"
    "LIST :ECHO=(0)  MODEL=(1)  RESULTS=(1)  MESSAGE=(2)  WARNING=(2)  (0:NO)\n"
    "FILE :CONV=(2)  GRAPH=(2)  MONITOR=(2)  HISTORY=(1)  ELEMENT=(0)  (0:NO)\n"
    "DISP :DISPLACEMENT MONITOR NODE NO.(%5d)  DIR=(%1d)    FACTOR=\n"
    "LOAD :APPLIED LOAD MONITOR NODE NO.(%5d)  DIR=(%1d)    FACTOR=\n"
    "UNIT :STRESS=(3) (1:kgf/cm**2  2:tf/m**2  3:N/mm**2=MPa)\n\n", node, dir, node, dir);
}

void column_axial_force(FILE *f, int startElm, const Geometry geo[], double Fc, int elmPp[])
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

void print_tail_template(FILE *f, struct loadNode load, int startElm, const Geometry geo[], double Fc, int elmPp[], int opt1)
{
    int dir = 1 + 2 * opt1;
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
    "TYPF :(  2)  MATJ(  2)  AXIS(   )\n"
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
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=-10      DIR(%1d)\n"
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=10       DIR(%1d)\n"
    " OUT :STEP  S(    2)-E(   11)-I(    1) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n"
    "\nEND\n", load.node1, dir, load.node2, dir);
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

void plot_node(FILE *f, int startNode, const StartEnd *point, const Geometry geo[], const int nodePp[])
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
                    return ;
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
                    return ;
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
                    return ;
                }
                print_COPYNODE(f, index, index + delt, nodePp[0], geo[2].mesh.length[i], nodePp[2], cnt, 2);
                index += cnt * nodePp[2];
            }
        }
    }
}

void define_quad(FILE *f, const Increment *pp, const StartEnd *point, const NodeElm *start, int dir1, int dir2, int typq)
{
    print_QUAD(f, start->elm, start->node, pp->node, dir1, dir2, typq);
    if((point->end[dir1] - point->start[dir1] - 1) == 0)
    {
        if((point->end[dir2] - point->start[dir2] - 1) > 0)
        {
            print_COPYELM(f, start->elm, 0, 0, pp->elm[dir2], pp->node[dir2], point->end[dir2] - point->start[dir2] - 1);
        }
    }
    else if((point->end[dir1] - point->start[dir1] - 1) > 0)
    {
        print_COPYELM(f, start->elm, 0, 0, pp->elm[dir1], pp->node[dir1], point->end[dir1] - point->start[dir1] - 1);
        if((point->end[dir2] - point->start[dir2] - 1) > 0)
        {
            print_COPYELM(f, start->elm, start->elm + (point->end[dir1] - point->start[dir1] - 1) * pp->elm[dir1], pp->elm[dir1], pp->elm[dir2], pp->node[dir2], point->end[dir2] - point->start[dir2] - 1);
        }
    }
}

void generate_fibar(FILE *f, const Increment *pp, const StartEnd *point, const NodeElm *start, const Geometry geo[], int typb)
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

void generate_quad(FILE *f, const Increment *pp, const StartEnd *point, const NodeElm *start, const Geometry geo[], int typq)
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
    define_quad(f, pp, point, start, dir1, dir2, typq);
}

void generate_hexa(FILE *f, const NodeElm *start, const Increment *pp, const StartEnd *point, const Geometry geo[], int typh)
{
    int delt;
    struct nodeElm index;
    index.node = start->node;
    index.elm  = start->elm;
    //節点定義
    plot_node(f, start->node, point, geo, pp->node);
    fprintf(f, "\n");
    //要素定義
    print_HEXA(f, start->elm, start->node, pp->node, typh);
    //要素コピー
    //x
    print_COPYELM(f, index.elm, 0, 0, pp->elm[0], pp->node[0], point->end[0] - point->start[0] - 1);
    //y
    delt = (point->end[0] - point->start[0] - 1) * pp->elm[0];
    print_COPYELM(f, index.elm, index.elm + delt, pp->elm[0], pp->elm[1], pp->node[1], point->end[1] - point->start[1] - 1);
    //z
    for(int i = 0; i < point->end[1] - point->start[1]; i++)
    {
        print_COPYELM(f, index.elm + pp->elm[1] * i, index.elm + delt + pp->elm[1] * i, pp->elm[0], pp->elm[2], pp->node[2], point->end[2] - point->start[2] - 1);
    }
}

void generate_line(FILE *f, int elm, int hexaNode, int beamNode, int nodePp, int set)
{
    const int elmPPz = 1;
    int index = elm;
    print_LINE(f, index, hexaNode, beamNode, nodePp);
    print_COPYELM(f, elm, 0, 0, elmPPz, nodePp, set);
}

int search_concrete_node(int start, const int pp[], int x, int y, int z, int boundaryZ)
{
    return start + pp[0] * x + pp[1] * y + pp[2] * (z + boundaryZ);
}

void add_reber(FILE *f, const Increment *pp, const ModelSize *rcs, const Geometry geo[], const NodeElm *rebarStartIndex, const int columnStartNode)
{
    const int dir  = 2;
    const int typb = 1;
    NodeElm  index;
    StartEnd point;
    index  = *rebarStartIndex;
    int rebarNum;
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs->rebar[i].cordi[0] < 0)
        {
            rebarNum = i;
            break;
        }
    }
    int fiberLineDelt = next_index((geo[2].boundary[4] - geo[2].boundary[1]) * rebarNum) - 1;
    fprintf(f, "\n----REBAR----");
    for(int j = 0; j < rebarNum; j++)
    {
        int pointX = coordinate_to_point(rcs->rebar[j].cordi[0], geo[0].mesh.length) - geo[0].boundary[2];
        int pointY = coordinate_to_point(rcs->rebar[j].cordi[1], geo[1].mesh.length) - geo[1].boundary[0];
        if(pointX > geo[0].boundary[3] - geo[0].boundary[2])
            pointX++;
        fprintf(f, "\n----\n");
        for(int i = 0; i < 2; i++)
        {
            point.start[i] = coordinate_to_point(rcs->rebar[j].cordi[i], geo[i].mesh.length);
            point.end  [i] = coordinate_to_point(rcs->rebar[j].cordi[i], geo[i].mesh.length);
        }
        for(int i = 1; i <= 3; i++)
        {
            index.node = (rebarStartIndex->node + j) + (geo[2].boundary[i] - geo[2].boundary[1] + i - 1) * pp->node[2];
            index.elm  = rebarStartIndex->elm + (geo[2].boundary[i] - geo[2].boundary[1]) * pp->elm[2] + (geo[2].boundary[4] - geo[2].boundary[1]) * j;
            int concreteNode = search_concrete_node(columnStartNode, pp->node, pointX, pointY, geo[2].boundary[i], i);
            point.start[2]   = geo[2].boundary[i];
            point.end  [2]   = geo[2].boundary[i + 1];
            generate_fibar(f, pp, &point, &index, geo, typb);
            generate_line(f, index.elm + fiberLineDelt, concreteNode, index.node, pp->node[2], geo[dir].boundary[i + 1] - geo[dir].boundary[i] - 1);
        }
    }
    for(int i = 2; i <= 3; i++)
    {
        index.node = rebarStartIndex->node + (geo[2].boundary[i] - geo[2].boundary[1] + i - 1) * pp->node[2];
        print_join(f, index.node - pp->node[2], index.node + rebarNum - 1 - pp->node[2], 1, index.node, index.node + rebarNum - 1, 1);
    }
}

void add_column_hexa(FILE *f, struct nodeElm startIndex, struct increment pp, const Geometry geo[])
{
    int typh[5] = {5, 1, 3, 1, 5};
    struct nodeElm index;
    struct startEnd point;
    index = startIndex;

    point.start[1] = geo[1].boundary[0];
    point.end  [1] = geo[1].boundary[2];
    for(int j = 0; j <= 4; j++)
    {
        fprintf(f, "\n----TYPH %1d----", typh[j]);
        point.start[2] = geo[2].boundary[j];
        point.end  [2] = geo[2].boundary[j + 1];
        for(int i = 2; i < 4; i++)
        {
            fprintf(f, "\n----\n");
            point.start[0] = geo[0].boundary[i];
            point.end  [0] = geo[0].boundary[i + 1];
            generate_hexa(f, &index, &pp, &point, geo, typh[j]);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1) * pp.node[0];
            index.elm  += (geo[0].boundary[i + 1] - geo[0].boundary[i])     * pp.elm [0];
        }
        index.node = startIndex.node + (geo[2].boundary[j + 1] + j + 1) * pp.node[2];
        index.elm  = startIndex.elm  +  geo[2].boundary[j + 1]          * pp.elm [2];
    }
}

void add_joint_quad(FILE *f, struct nodeElm startIndex, const Geometry geo[], struct increment pp, const ModelSize rcs, int columnHexaNode)
{
    int film;
    int typq[5] = {0};
    NodeElm  index;
    StartEnd point;
    Increment jointPp = pp;
    jointPp.elm[0] = 1;
    jointPp.elm[1] = pp.elm[1] + 4;
    jointPp.elm[2] = jointPp.elm[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 2);

    int filmStart = startIndex.elm + (geo[0].boundary[4] - geo[0].boundary[2] + 4) * (geo[1].boundary[2] - geo[1].boundary[0] + 2) * (geo[2].boundary[3] - geo[2].boundary[2] + 4);
    filmStart = next_index(filmStart);

    printf("film -> %d\n", filmStart);
    index = startIndex;
    const int concreteSteelDelt = startIndex.node - (columnHexaNode + (geo[2].boundary[2] + 2) * pp.node[2]);
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
                    return;
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
        print_FILM(f, film, index.node - delt - pp.node[2], index.node, pp.node, 0, 1, 1);
        print_FILM(f, film + jointPp.elm[2], index.node - delt, index.node, pp.node, 1, 0, 1);
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
    film = filmStart + (geo[0].boundary[3] - geo[0].boundary[2] + 1) * jointPp.elm[0] + jointPp.elm[1] + 2 * jointPp.elm[2];
    print_etyp(f, film, film + (geo[1].boundary[2] - 1) * jointPp.elm[1], jointPp.elm[1], 2, jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    film = filmStart + (geo[0].boundary[3] - geo[0].boundary[2] + 2) * jointPp.elm[0] + jointPp.elm[1] + 2 * jointPp.elm[2];
    print_etyp(f, film, film + (geo[1].boundary[2] - 1) * jointPp.elm[1], jointPp.elm[1], 2, jointPp.elm[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
}

void hexa_beam(FILE *f, struct nodeElm startIndex, struct increment pp, const Geometry geo[])
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

void quad_beam(FILE *f, struct nodeElm startIndex, struct increment ppColumn, struct increment ppBeam, const Geometry geo[], int jointStartNode)
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

void joint_nodes(FILE *f, int startNode, const Geometry geo[], const ModelSize rcs, int pp[])
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

void set_pin(FILE *f, int startNode, const Geometry geo[], int pp[], int opt1)
{
    int rest = 99 * opt1 + 1;
    int dir  = 2 - 2 * opt1;
    fprintf(f, "\n----\n");
    int node = startNode;
    int delt = (geo[1].boundary[2] - geo[1].boundary[1 - opt1]) * pp[1];
    print_rest(f, node, node + delt, pp[1], rest, pp[dir], geo[dir].boundary[3 + opt1] - geo[dir].boundary[2] + opt1);
    node += (geo[2 * opt1].boundary[6 - opt1] + 4 * opt1)* pp[2 * opt1];
    print_rest(f, node, node + delt, pp[1], rest, pp[dir], geo[dir].boundary[3 + opt1] - geo[dir].boundary[2] + opt1);
}

void set_roller(FILE *f, int startNode, struct loadNode load, const Geometry geo[], int pp[], int opt1, int span)
{
    int dir = opt1 * 2;
    int sub = dir + 1;
    int loadNode[2];
    if(load.node1 < load.node2)
    {
        loadNode[0] = load.node1;
        loadNode[1] = load.node2;
    }else{
        loadNode[0] = load.node2;
        loadNode[1] = load.node1;
    }
    int delt = (geo[1].boundary[2] - geo[1].boundary[opt1]) * pp[1];
    int opposite = 2 - opt1 * 2;
    for(int j = 0; j <= 1; j++)
    {
        for(int i = geo[dir].boundary[2]; i <= geo[dir].boundary[4 - opt1]; i++)
        {
            int node = startNode + (i - geo[dir].boundary[2]) * pp[dir] + j * (geo[opposite].boundary[5 + opt1] + 4 * (1 - opt1)) * pp[opposite];
            if(i > geo[dir].boundary[3])
            {
                node += (1 - opt1) * pp[dir];
            }
            if((node + delt) != loadNode[j])
            {
                print_sub1(f, node, node + delt, pp[1], sub, loadNode[j], sub);
            }
            else
            {
                print_sub1(f, node, node + delt - pp[1], pp[1], sub, loadNode[j], sub);
            }
        }
    }
}

void cut_surface(FILE *f, const Geometry geo[], int columnStart, int jointStart, int beamStart, int columnPp[], int beamPp[])
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

void add_typh(FILE *f, const Geometry geo[], const ModelSize rcs, int columnStart, int columnPp[])
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
        beam         : 梁
        fibar        : BEAM要素
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
    Increment columnPp;
    Increment beamPp;
    LoadNode  roller;
    LoadNode  pin;

    printf("\n[START]\n");

    FILE *fin = fopen(inputFileName, "r");
    if(fin == NULL){
        printf("[ERROR] %s cant open.\n", inputFileName);
        return ;
    }
    const ModelSize rcs = load_size(fin);
    const Geometry  modelGeometry[XYZ] = {set_geometry(fin, 0, &rcs), set_geometry(fin, 1, &rcs), set_geometry(fin, 2, &rcs)};
    fclose(fin);
    
    //データチェック
    printf("\n------------------[ DATE CHECK ]------------------\n\n");
    printf("non\n----\n");
    
    //試験体情報表示
    console_model_sizes(rcs);

    //メッシュ情報表示
    console_mesh(modelGeometry);
    printf("\n");

    //境界線情報表示
    console_boundary(modelGeometry);

    //インクリメント
    get_increment(&columnPp, &beamPp, modelGeometry);
    console_increment(&columnPp);
    console_increment(&beamPp);

    //
    NodeElm   columnHexa;
    columnHexa.node = startNode;
    columnHexa.elm  = startElm;

    NodeElm   columnFibar;
    columnFibar.node = columnHexa.node - 1;
    columnFibar.node += (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2] + 2)
                      * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 1)
                      * (modelGeometry[2].boundary[5] - modelGeometry[2].boundary[0] + 5);
    columnFibar.node = next_index(columnFibar.node);
    columnFibar.elm  = columnHexa.elm - 1;
    columnFibar.elm += (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2])
                     * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0])
                     * (modelGeometry[2].boundary[5] - modelGeometry[2].boundary[0]);
    columnFibar.elm  = next_index(columnFibar.elm);

    NodeElm   jointQuad;
    int elmNum = 0;
    jointQuad.node  = columnFibar.node - 1;
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs.rebar[i].cordi[0] < 0)
        {
            jointQuad.node += i - 1;
            elmNum         += i - 1;
            break;
        }
    }
    jointQuad.node += (modelGeometry[2].boundary[4] - modelGeometry[2].boundary[1] + 3) * columnPp.node[2];
    jointQuad.node  = next_index(jointQuad.node);
    elmNum         += (modelGeometry[2].boundary[4] - modelGeometry[2].boundary[1]) * columnPp.elm[2];
    elmNum          = next_index(elmNum) - 1;
    jointQuad.elm  = columnFibar.elm + elmNum * 2;

    NodeElm   beamHexa;
    beamHexa.node = jointQuad.node - 1;
    beamHexa.node += (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2] + 1)
                      * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 1)
                      * (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1);
    beamHexa.node = next_index(beamHexa.node);
    elmNum = 0;
    elmNum += (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2] + 4)
                      * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 2)
                      * (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 4);
    elmNum = next_index(elmNum) - 1;
    beamHexa.elm = jointQuad.elm + elmNum * 2;

    get_load_node(&pin, &roller, columnHexa.node, beamHexa.node, columnPp.node, beamPp.node, modelGeometry, rcs.column.span, opt1);

    //node
    FILE *fnode = fopen("node.txt", "w");
    if(fnode != NULL)
    {
        fprintf(fnode, "--\nparts   node      incX     Y     Z\n");
        fprintf(fnode, "column %5d  %5d %5d %5d\n", columnHexa.node, columnPp.node[0], columnPp.node[1], columnPp.node[2]);
        fprintf(fnode, "joint  %5d  %5d %5d %5d\n", jointQuad.node, columnPp.node[0], columnPp.node[1], columnPp.node[2]);
        fprintf(fnode, "beam   %5d  %5d %5d %5d\n", beamHexa.node, beamPp.node[0], beamPp.node[1], beamPp.node[2]);
        fprintf(fnode, "--\nroller %5d %5d\n", roller.node1, roller.node2);
        fprintf(fnode, "pin    %5d %5d\n", pin.node1, pin.node2);
        fprintf(fnode, "--\nweb panel\n");
        int setY = modelGeometry[1].boundary[2] * columnPp.node[1];
        int setZ = (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * columnPp.node[2];
        fprintf(fnode, "%05d --- %05d --- %05d\n", jointQuad.node + setY + setZ, jointQuad.node + (modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2]) * columnPp.node[0] + setY + setZ, jointQuad.node + (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2]) * columnPp.node[0] + setY + setZ);
        fprintf(fnode, "    |                   |\n");
        setZ = (coordinate_to_point(rcs.column.span / 2, modelGeometry[2].mesh.length) - modelGeometry[2].boundary[2]) * columnPp.node[2];
        fprintf(fnode, "%05d               %05d\n", jointQuad.node + setY + setZ, jointQuad.node + (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2]) * columnPp.node[0] + setY + setZ);
        fprintf(fnode, "    |                   |\n");
        fprintf(fnode, "%05d --- %05d --- %05d\n", jointQuad.node + setY, jointQuad.node + (modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2]) * columnPp.node[0] + setY, jointQuad.node + (modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2]) * columnPp.node[0] + setY);
        fclose(fnode);
    }
    else
    {
        printf("[ERROR] \n");
    }
    
    //ファイルオープン
    FILE *fout = fopen(OUT_FILE_NAME,"w");
    if(fout == NULL)
    {
        printf("[ERROR] out.ffi cant open.\n");
        return ;
    }

    //雛形
    print_head_template(fout, roller, opt1);    
    printf("\n------------------[ COLUMN ]------------------\n");

    //柱六面体要素
    console_index(columnHexa, "concrete");
    add_column_hexa(fout, columnHexa, columnPp, modelGeometry);

    //柱線材要素
    console_index(columnFibar, "rebar");
    add_reber(fout, &columnPp, &rcs, modelGeometry, &columnFibar, columnHexa.node);

    printf("\n------------------[ BEAM ]------------------\n");
    //四辺形要素
    console_index(jointQuad, " column quad");
    add_joint_quad(fout, jointQuad, modelGeometry, columnPp, rcs, columnHexa.node);

    //六面体要素
    console_index(beamHexa, "hexa beam");
    hexa_beam(fout, beamHexa, beamPp, modelGeometry);

    //四辺形要素
    quad_beam(fout, beamHexa, columnPp, beamPp, modelGeometry, jointQuad.node);
    
    //
    joint_nodes(fout, startNode, modelGeometry, rcs, columnPp.node);

    //柱加力or梁加力
    if(opt1 == 0)//柱加力
    {
        set_pin(fout, beamHexa.node, modelGeometry, beamPp.node, opt1);
        set_roller(fout, columnHexa.node, roller, modelGeometry, columnPp.node, opt1, rcs.beam.span);
    }else if(opt1 == 1)//梁加力
    {
        set_pin(fout, columnHexa.node, modelGeometry, columnPp.node, opt1);
        set_roller(fout, beamHexa.node, roller, modelGeometry, beamPp.node, opt1, rcs.column.span);
    }

    cut_surface(fout, modelGeometry, startNode, jointQuad.node, beamHexa.node, columnPp.node, beamPp.node);
    //六面体要素番号
    add_typh(fout, modelGeometry, rcs, startElm, columnPp.elm);

    print_tail_template(fout, roller, startElm, modelGeometry, rcs.column.Fc, columnPp.elm, opt1);
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
    /*
        command [options] [arguments]
    */
    int         optchar;
    int         opt1 = 0;
    const char* inputFile = NULL;

    if(argc < 2)
    {
        show_usage();
        return 1;
    }
    while((optchar = getopt(argc, argv, OPTCHAR)) != -1)
    {
        printf("opt\n");
        switch(optchar){
            case 'b': //beam 梁加力
                opt1 = 1;
                printf("option b\n");
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