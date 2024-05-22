#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#define TEMPLATE_FILE_NAME         "template.txt"
#define LOADING_TEMPLATE_FILE_NAME "step_disp.txt"
#define OUT_FILE_NAME              "out.ffi"
#define OPTCHAR                    "tlbfs:o:v"
#define MESH_ARRAY_MAX 64 //注意が必要
#define STEP_DISP_MAX  64
#define REBAR_MAX      32 
#define BOUNDARY_MAX   8 //確定
#define XYZ            3    
#define JIG            1
#define FIRST_NODE     1
#define FIRST_ELM      1

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

typedef struct
{
    int hexa;
    int fiber;
} columnNode;

typedef struct
{
    int hexa;
    int fiber;
    int line;
} columnElm;

typedef struct
{
    columnNode node;
    columnElm  elm;
} column;

typedef struct
{
    int quad;
} jointNode;

typedef struct
{
    int quad;
    int film;
} jointElm;

typedef struct
{
    jointNode node;
    jointElm  elm;
} joint;

typedef struct
{
    int node;
    int elm;
} beam;

typedef struct
{
    int    step;
    double disp;
} StepDisp;

void read_int_csv(FILE *f, int array[])
{
    //()で囲まれた整数型csvデータを読み込む
    //最後尾の要素は-999を格納する
    //ファイルポインタを改行後に移動
    int cnt = 0;
    if(fgetc(f) == '(')
    {
        fscanf(f, "%d[^,)]", &array[cnt++]);
        while (fgetc(f) != ')')
        {
            fscanf(f, "%d[^,)]", &array[cnt++]);
        }
        array[cnt] = -999;
    }
    else
    {
        printf("error\n");
        return;
    }
    while((fgetc(f)) != '\n'){}
}

void read_double_csv(FILE *f, double array[])
{
    //()で囲まれた実数型csvデータを読み込む
    //最後尾の要素は-999を格納する
    //ファイルポインタを改行後に移動
    int cnt = 0;
    if(fgetc(f) == '(')
    {
        fscanf(f, "%lf[^,)]", &array[cnt++]);
        while (fgetc(f) != ')')
        {
            fscanf(f, "%lf[^,)]", &array[cnt++]);
        }
        array[cnt] = -999.0;
    }
    else
    {
        printf("error\n");
        return;
    }
    while((fgetc(f)) != '\n'){}
}


void read_csv(FILE *f, Mesh *mesh)
{
    //()で囲まれたcsvを読み取る
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

void read_loading_step(const char *loadingStepFile, StepDisp stepDisp[])
{
    int step[STEP_DISP_MAX];
    double disp[STEP_DISP_MAX];
    FILE *f = fopen(loadingStepFile, "r");
    if(f == NULL)
    {
        printf("error\n");
        return;
    }
    else
    {
        //stepデータ読込み
        fscanf(f, "step");
        read_int_csv(f, step);
        //dispデータ読込み
        fscanf(f, "disp");
        read_double_csv(f, disp);
        fclose(f);
    }
    if(step[0] <= 1)
    {
        //step1は軸力導入
        printf("error\n");
        return;
    }
    for(int i = 0; ; i++)
    {
        stepDisp[i].step = step[i];
        stepDisp[i].disp = disp[i];
        if(step[i] < 0)
        {
            break;
        }
    }
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
        boundary[1] = JIG;
        boundary[2] = coordinate_to_point((rcs->column.span - rcs->beam.depth) / 2, mesh->length);
        boundary[3] = coordinate_to_point((rcs->column.span + rcs->beam.depth) / 2, mesh->length);
        boundary[4] = mesh->number - JIG;
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
    node.node2 = node.node1 + (geo[2].boundary[5] + 2) * columnPp[2];
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

void get_column_increment(Increment *column, const Geometry geo[])
{
    //柱
    column->node[0] = 1;
    column->node[1] = geo[0].boundary[4] - geo[0].boundary[2] + 2;
    column->node[2] = column->node[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 1);
    column->elm [0] = 1;
    column->elm [1] = geo[0].boundary[4] - geo[0].boundary[2];
    column->elm [2] = column->elm[1] * (geo[1].boundary[2] - geo[1].boundary[0]);
}

void get_joint_increment(Increment *column, Increment *joint, const Geometry geo[])
{
    //接合部
    *joint = *column;
    joint->elm[1] = column->elm[1] + 4;
    joint->elm[2] = joint->elm[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 2);
}

void get_beam_increment(Increment *beam, const Geometry geo[])
{
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

void console_numbers(const column *columnNumbers, const joint *jointNumbers, const beam *beamNumbers)
{
    printf("\n------------------[ NUMBERS ]------------------\n");
    printf("column\n");
    printf("     :  hexa  fiber   line\n");
    printf("node : %5d  %5d\n", columnNumbers->node.hexa, columnNumbers->node.fiber);
    printf("elm  : %5d  %5d  %5d\n", columnNumbers->elm.hexa, columnNumbers->elm.fiber, columnNumbers->elm.line);
    printf("\njoint\n");
    printf("     :  quad   film\n");
    printf("node : %5d\n", jointNumbers->node.quad);
    printf("elm  : %5d  %5d\n", jointNumbers->elm.quad, jointNumbers->elm.film);
    printf("\nbeam\n");
    printf("     : number\n");
    printf("node : %5d\n", beamNumbers->node);
    printf("elm  : %5d\n", beamNumbers->elm);
}

void console_step_disp(StepDisp stepDisp[])
{
    printf("\n------------------[ STEPDISP ]------------------\n");
    printf("step :");
    for(int i = 0; ; i++)
    {
        printf(" %8d", stepDisp[i].step);
        if(stepDisp[i].step < 0)
        {
            printf("\n");
            break;
        }
    }
    printf("disp :");
    for(int i = 0; ; i++)
    {
        printf(" %8.2f", stepDisp[i].disp);
        if(stepDisp[i].step < 0)
        {
            printf("\n");
            break;
        }
    }
}

//頭のひな形
void print_head_template(FILE *f, struct loadNode load, int opt1, int lastStep)
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
    "EXEC :STEP (    1)-->(%5d)  ELASTIC=( ) CHECK=(1) POST=(1) RESTART=( )\n"
    "LIST :ECHO=(0)  MODEL=(1)  RESULTS=(1)  MESSAGE=(2)  WARNING=(2)  (0:NO)\n"
    "FILE :CONV=(2)  GRAPH=(2)  MONITOR=(2)  HISTORY=(1)  ELEMENT=(0)  (0:NO)\n"
    "DISP :DISPLACEMENT MONITOR NODE NO.(%5d)  DIR=(%1d)    FACTOR=\n"
    "LOAD :APPLIED LOAD MONITOR NODE NO.(%5d)  DIR=(%1d)    FACTOR=\n"
    "UNIT :STRESS=(3) (1:kgf/cm**2  2:tf/m**2  3:N/mm**2=MPa)\n\n", lastStep, node, dir, node, dir);
}

void print_column_axial_force(FILE *f, int startElm, const Geometry geo[], double Fc, int elmPp[])
{
    int elm;
    double rate = 0.2;
    double unit = rate * Fc;
    fprintf(f, "\n----+----\n");
    fprintf(f,"\nSTEP :UP TO NO.(    1)   MAXIMUM LOAD INCREMENT=         CREEP=( )(0:NO)\n");
    for(int i = 0; i < geo[1].boundary[2]; i++)
    {
        elm = startElm + i * elmPp[1];
        fprintf(f,"  UE :ELM   S(%5d)-E(%5d)-I(%5d)     UNIT=%-9.4fDIR(3)  FACE(1)\n", elm, elm + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * elmPp[0], elmPp[0], unit);
    }
    for(int i = 0; i < geo[1].boundary[2]; i++)
    {
        elm = startElm + i * elmPp[1] + (geo[2].boundary[5] - geo[2].boundary[1]) * elmPp[2];
        fprintf(f,"  UE :ELM   S(%5d)-E(%5d)-I(%5d)     UNIT=-%-8.4fDIR(3)  FACE(2)\n", elm, elm + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * elmPp[0], elmPp[0], unit);
    }
    fprintf(f," OUT :STEP  S(    1)-E(     )-I(     ) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n\n");
}

void print_type_mat(FILE *f)
{
    fprintf(f,
    "\n----+----\n"
    "TYPH :(  1)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  2)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  3)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  4)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  5)  MATS(  1)  AXIS(   )\n"
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
    "AXIS :(  1)  TYPE=(1) (1:GLOBAL 2:ELEMENT 3:INPUT 4:CYLINDER 5:SPHERE)\n");
}

void print_test_step(FILE *f, struct loadNode load, int opt1)
{
    int dir = 0;
    if(opt1 == 0)
    {
        //柱加力
        dir = 1;
    }
    else if(opt1 == 1)
    {
        //梁加力
        dir = 3;
    }
    fprintf(f,
    "\n"
    "STEP :UP TO NO.(   11)   MAXIMUM LOAD INCREMENT=         CREEP=(0)(0:NO)\n"
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=-10      DIR(%1d)\n"
    "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=10       DIR(%1d)\n"
    " OUT :STEP  S(    2)-E(   11)-I(    1) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n"
    "\nEND\n", load.node1, dir, load.node2, dir);
}

void print_loading_step(FILE *f, struct loadNode load, int opt1, StepDisp stepDisp[])
{
    int dir = 0;
    if(opt1 == 0)
    {
        //柱加力
        dir = 1;
    }
    else if(opt1 == 1)
    {
        //梁加力
        dir = 3;
    }
    for(int i = 0; stepDisp[i].step > 0;i++)
    {
        fprintf(f,
        "\n"
        "STEP :UP TO NO.(%5d)   MAXIMUM LOAD INCREMENT=         CREEP=(0)(0:NO)\n"
        "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=%-9.2fDIR(%1d)\n"
        "  FN :NODE  S(%5d)-E(     )-I(     )     DISP=%-9.2fDIR(%1d)\n"
        " OUT :STEP  S(%5d)-E(     )-I(     ) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n"
        , stepDisp[i].step, load.node1, -1 * stepDisp[i].disp, dir, load.node2, stepDisp[i].disp, dir, stepDisp[i].step);
    }
    fprintf(f, "\nEND\n");
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

/*ある格子点間で節点コピーする*/
void auto_copynode(FILE *f, int start, int end, int inter, int startPoint, int endPoint, const double length[], const int nodePp, int dir)
{
    int index = start;
    int delt = end - start;
    for(int i = startPoint, cnt = 0; i < endPoint; i += cnt)
    {
        cnt = count_consecutive(i, endPoint, length);
        if(cnt < 0)
        {
            printf("[ERROR] X\n");
            return ;
        }
        if(end == 0)
        {
            print_COPYNODE(f, index, 0, 0, length[i], nodePp, cnt, dir);
        }
        else if(end != 0)
        {
            print_COPYNODE(f, index, index + delt, inter, length[i], nodePp, cnt, dir);
        }
        index += cnt * nodePp;
    }
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

int search_concrete_node(int conStart, const int pp[], const Geometry geo[], int x, int y, int z)
{
    //節点が重なる位置の場合、大きい方の節点番号を返す。
    //節点番号は要素定義で使われる、その要素は境界線位置にあるので。
    if(x >= geo[0].boundary[3] - geo[0].boundary[2])
    {
        x++;
    }
    if(y >= geo[1].boundary[2])
    {
        y++;
    }
    for(int i = 2; i <= 3; i++)
    {
        if(z < geo[2].boundary[i])
        {
            break;
        }
        z++;
    }
    return conStart + pp[0] * x + pp[1] * y + pp[2] * z;
}

void add_reber(FILE *f, const Increment *pp, const ModelSize *rcs, const Geometry geo[], const column *head)
{
    const int dir  = 2; //方向
    const int typb = 1; //要素タイプ
    const int rebarElmNum = geo[2].boundary[4] - geo[2].boundary[1]; //1本の主筋の要素数
    int rebarNum;
    double cordi[3];

    fprintf(f, "\n----REBAR----\n");
    //底部分節点定義
    cordi[2] = point_to_coordinate(geo[2].boundary[1], geo[2].mesh.length);
    for(int i = 0; rcs->rebar[i].cordi[0] > 0; i++)
    {
        int node = head->node.fiber + i;
        cordi[0] = rcs->rebar[i].cordi[0];
        cordi[1] = rcs->rebar[i].cordi[1];
        print_NODE(f, node, cordi);
        rebarNum = i;
    }
    //節点コピー
    int start = head->node.fiber;
    int end   = start + rebarNum;
    auto_copynode(f, start, end, 1, geo[dir].boundary[1], geo[dir].boundary[2], geo[dir].mesh.length, pp->node[dir], dir);

    for(int i = 2; i <= 3; i++)
    {
        //節点がかさなる部分
        start += (geo[dir].boundary[i] - geo[dir].boundary[i - 1]) * pp->node[dir];
        end   += (geo[dir].boundary[i] - geo[dir].boundary[i - 1]) * pp->node[dir];
        print_COPYNODE(f, start, end, 1, 0, pp->node[dir], 1, dir);
        //節点コピー
        start += pp->node[dir];
        end   += pp->node[dir];
        auto_copynode(f, start, end, 1, geo[dir].boundary[i], geo[dir].boundary[i + 1], geo[dir].mesh.length, pp->node[dir], dir);
    }
    for(int j = 1; j <= 3; j++)//高さ方向
    {
        //要素定義
        for(int i = 0; rcs->rebar[i].cordi[0] > 0; i++)
        {
            int rebarNode = head->node.fiber + (geo[dir].boundary[j] - geo[dir].boundary[1] + j - 1) * pp->node[dir] + i;
            int conNode   = search_concrete_node(head->node.hexa, pp->node, geo,
                                                coordinate_to_point(rcs->rebar[i].cordi[0], geo[0].mesh.length) - geo[0].boundary[2],
                                                coordinate_to_point(rcs->rebar[i].cordi[1], geo[1].mesh.length),
                                                geo[2].boundary[j]);
            int rebarElm = head->elm.fiber + rebarElmNum * i + geo[dir].boundary[j] - geo[dir].boundary[1];
            int lineElm  = head->elm.line  + rebarElmNum * i + geo[dir].boundary[j] - geo[dir].boundary[1];
            print_BEAM(f, rebarElm, rebarNode, pp->node[dir], typb);
            print_LINE(f, lineElm, rebarNode, conNode, pp->node[dir]);
        }
        //要素コピー
        start = head->elm.fiber + geo[dir].boundary[j] - geo[dir].boundary[1];
        end   = start + rebarNum * rebarElmNum;
        print_COPYELM(f, start, end, rebarElmNum, 1, pp->node[dir], geo[dir].boundary[j + 1] - geo[dir].boundary[j] - 1);
        start = head->elm.line + geo[dir].boundary[j] - geo[dir].boundary[1];
        end   = start + rebarNum * rebarElmNum;
        print_COPYELM(f, start, end, rebarElmNum, 1, pp->node[dir], geo[dir].boundary[j + 1] - geo[dir].boundary[j] - 1);
    }
    //節点結合
    for(int i = 2; i <= 3; i++)
    {
        int node = head->node.fiber + (geo[dir].boundary[i] - geo[dir].boundary[1] + i - 2) * pp->node[2];
        print_join(f, node, node + rebarNum, 1, node + pp->node[dir], node + pp->node[dir] + rebarNum, 1);
    }
}

void add_column_hexa(FILE *f, const column *head, struct increment pp, const Geometry geo[])
{
    const int bou[4] = {0, 2, 3, 5};
    int typh[3] = {1, 3, 1};
    struct nodeElm index;
    struct startEnd point;
    //y方向
    point.start[1] = geo[1].boundary[0];
    point.end  [1] = geo[1].boundary[2];
    for(int j = 0; j <= 2; j++)
    {
        fprintf(f, "\n----TYPH %1d----", typh[j]);
        //z方向
        point.start[2] = geo[2].boundary[bou[j]];
        point.end  [2] = geo[2].boundary[bou[j + 1]];
        for(int i = 2; i <= 3; i++)
        {
            fprintf(f, "\n----\n");
            //x方向
            point.start[0] = geo[0].boundary[i];
            point.end  [0] = geo[0].boundary[i + 1];
            //節点、要素番号
            index.node = head->node.hexa + (geo[0].boundary[i] - geo[0].boundary[2] + i - 2) * pp.node[0] + (geo[2].boundary[bou[j]] + j) * pp.node[2];
            index.elm  = head->elm.hexa  + (geo[0].boundary[i] - geo[0].boundary[2])         * pp.elm [0] +  geo[2].boundary[bou[j]]      * pp.elm [2];
            generate_hexa(f, &index, &pp, &point, geo, typh[j]);
        }
    }
}

void add_joint_quad(FILE *f, const column *columnHeads, const joint *jointHeads, const Geometry geo[], struct increment pp, const ModelSize rcs)
{
    int film;
    int typq[5] = {0};
    NodeElm  index;
    StartEnd point;
    Increment jointPp = pp;
    jointPp.elm[0] = 1;
    jointPp.elm[1] = pp.elm[1] + 4;
    jointPp.elm[2] = jointPp.elm[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 2);

    int filmStart = jointHeads->elm.film;

    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
    const int concreteSteelDelt = jointHeads->node.quad - (columnHeads->node.hexa + (geo[2].boundary[2] + 1) * pp.node[2]);
    fprintf(f, "\n----JOINT QUAD----\n");
    //x直交面
    fprintf(f, "\n----yz\n");
    set_start_end(&point, 0, 2, geo[1].boundary, 1);
    set_start_end(&point, 2, 3, geo[2].boundary, 2);
    index.elm  = jointHeads->elm.quad + jointPp.elm[1] + jointPp.elm[2];
    typq[2] = 9;
    typq[3] = 5;
    typq[4] = 13;
    for(int i = 2; i <= 4; i++)
    {
        index.node = jointHeads->node.quad + (geo[0].boundary[i] - geo[0].boundary[2]) * pp.node[0];
        index.elm  = jointHeads->elm.quad  + (geo[0].boundary[i] - geo[0].boundary[2] + i - 2) * jointPp.elm[0] + jointPp.elm[1] + jointPp.elm[2];
        set_start_end(&point, i, i, geo[0].boundary, 0);
        generate_quad(f, &jointPp, &point, &index, geo, typq[i]);
    }
    //film要素
    fprintf(f, "\n----\n");
    film = filmStart + jointPp.elm[1] + 2 * jointPp.elm[2];
    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
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
    fprintf(f, "\n----zx\n");
    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
    typq[0] = 11;
    typq[2] = 2;
    set_start_end(&point, 2, 3, geo[2].boundary, 2);
    for(int j = 0; j <= 2; j += 2)
    {
        for(int i = 2; i <= 3; i++)
        {
            index.node = jointHeads->node.quad + (geo[0].boundary[i] - geo[0].boundary[2] + 1) * pp.node[0] + (geo[1].boundary[j]) * pp.node[1];
            set_start_end(&point, i, i + 1, geo[0].boundary, 0);
            point.start[0]++;
            point.end[0]--;
            set_start_end(&point, j, j, geo[1].boundary, 1);
            plot_node(f, index.node, &point, geo, pp.node);
            index.elm  = jointHeads->elm.quad + (geo[0].boundary[i] - geo[0].boundary[2] + 1 + i - 2) * jointPp.elm[0] + (geo[1].boundary[j] + j / 2) * jointPp.elm[1] + jointPp.elm[2];
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
    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
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
        index.node = jointHeads->node.quad + (geo[0].boundary[3] - geo[0].boundary[2]) * jointPp.elm[0];
        delt -= pp.node[0];
    }

    //z直交面
    fprintf(f, "\n----xy\n");
    for(int k = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length); k < geo[1].boundary[2]; k++)
    {
        index.node = jointHeads->node.quad + k * pp.node[1];
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
            index.node = jointHeads->node.quad + (j - geo[0].boundary[2]) * pp.node[0];
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
        index.node = jointHeads->node.quad + j * pp.node[1];
        index.elm  = jointHeads->elm.quad  + 1 * jointPp.elm[0] + (j + 1) * jointPp.elm[1];
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

        index.node = jointHeads->node.quad + (i - geo[0].boundary[2]) * pp.node[0];
        index.elm  = jointHeads->elm.quad  + (i - geo[0].boundary[2] + 1) * jointPp.elm[0] + 1 * jointPp.elm[1];
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
    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
    delt = concreteSteelDelt;
    for(int i = 2; i <= 3; i++)
    {
        index.node = jointHeads->node.quad + (geo[0].boundary[i] - geo[0].boundary[2]) * pp.node[0] + geo[1].boundary[1] * pp.node[1];
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
    index.node = jointHeads->node.quad;
    index.elm  = jointHeads->elm.quad;
    delt = concreteSteelDelt;
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); i++)
    {
        index.node = jointHeads->node.quad + (i - geo[0].boundary[2]) * pp.node[0];
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
    index.elm = jointHeads->elm.quad + (geo[1].boundary[1] + 1) * jointPp.elm[1] + jointPp.elm[2];
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
    index.elm = jointHeads->elm.quad + (coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length) - geo[0].boundary[2] + 1) * jointPp.elm[0] + jointPp.elm[2];
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

void hexa_beam(FILE *f, const beam *beamHeads, struct increment pp, const Geometry geo[])
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
        index.node = beamHeads->node + geo[0].boundary[i] * pp.node[0];
        index.elm  = beamHeads->elm  + geo[0].boundary[i] * pp.node[0] + pp.elm[2];
        point.start[0] = geo[0].boundary[i];
        point.end  [0] = geo[0].boundary[i + 1];
        generate_hexa(f, &index, &pp, &point, geo, typh);
    }
}

void quad_beam(FILE *f, const beam *beamHeads, struct increment ppColumn, struct increment ppBeam, const Geometry geo[], const joint *jointHeads)
{
    int flangeNum = geo[1].boundary[2] - geo[1].boundary[1];
    const int typq[4] = {0, 0, 4, 3};
    int jointNode = jointHeads->node.quad;
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
            index.node = beamHeads->node + (geo[0].boundary[j] + 1) * ppBeam.node[0] + (geo[2].boundary[i] - geo[2].boundary[2]) * ppBeam.node[2];
            plot_node(f, index.node, &point, geo, ppBeam.node);
        }
        point.start[1] = geo[1].boundary[2];
        point.start[2] = geo[2].boundary[2] + 1;
        point.end  [2] = geo[2].boundary[3] - 1;
        index.node = beamHeads->node + (geo[0].boundary[j] + 1) * ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1] + ppBeam.node[2];
        plot_node(f, index.node, &point, geo, ppBeam.node);
    }
    //要素定義
    //接合部分
    quadDelt    = (geo[0].boundary[4] - geo[0].boundary[2] + 2) * ppBeam.node[0];
    jointDelt   = (geo[0].boundary[4] - geo[0].boundary[2])     * ppColumn.node[0];
    elmDelt     = (geo[0].boundary[4] - geo[0].boundary[2] + 1) * ppBeam.elm[0];

    index.node  = beamHeads->node + (geo[0].boundary[2] - 1) * ppBeam.node[0];
    index.elm   = beamHeads->elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0];
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

    jointNode  = jointHeads->node.quad;
    index.node = beamHeads->node + (geo[0].boundary[2] - 1) * ppBeam.node[0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = beamHeads->elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm [1] + ppBeam.elm [2];
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
        index.node = beamHeads->node + ppBeam.node[0] + (geo[2].boundary[i] - geo[2].boundary[2]) * ppBeam.node[2];
        index.elm  = beamHeads->elm  + ppBeam.elm [0] + (geo[2].boundary[i] - geo[2].boundary[2] + i - 2) * ppBeam.elm[2];
        print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 1, typq[i]);
        print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 1, typq[i]);    
        print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
        if(flangeNum > 1)
        {
            print_COPYELM(f, index.elm, index.elm + (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0], ppBeam.elm[0], ppBeam.elm[1], ppBeam.node[1], flangeNum - 1);
            print_COPYELM(f, index.elm + elmDelt, index.elm + elmDelt + (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0], ppBeam.elm[0], ppBeam.elm[1], ppBeam.node[1], flangeNum - 1);
        }
    }
    index.node = beamHeads->node + ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = beamHeads->elm  + ppBeam.elm [0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 2, 1);
    print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 2, 1);    
    print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
    
    index.elm = beamHeads->elm + ppBeam.elm[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    elmDelt = (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    index.elm += (geo[0].boundary[4] - geo[0].boundary[1] + 1) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
}

void merge_nodes(FILE *f, int concreteStartNode, const Geometry geo[], const ModelSize rcs, int pp[])
{
    int node;
    //x直交面
    fprintf(f, "\n----yz\n");
    int delt = (geo[2].boundary[2] - geo[2].boundary[0]) * pp[2];
    for(int i = 0; i <= geo[1].boundary[2]; i++)
    {
        //結合先節点
        node  = concreteStartNode + (geo[0].boundary[3] - geo[0].boundary[2]) * pp[0] + i * pp[1];
        print_join(f, node, node + delt, pp[2], node + pp[0], node + delt + pp[0], pp[2]);
        node += (geo[2].boundary[3] + 2) * pp[2];
        print_join(f, node, node + delt, pp[2], node + pp[0], node + delt + pp[0], pp[2]);
    }
    //接合部
    fprintf(f, "\n----xy\n");
    delt = geo[1].boundary[1] * pp[1];
    for(int i = geo[0].boundary[2]; i <= geo[0].boundary[4]; i++)
    {
        if(i <= coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length) || i >= coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length))
        {
            //結合先節点
            node = concreteStartNode + (i - geo[0].boundary[2]) * pp[0] + (geo[2].boundary[2]) * pp[2];
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
    //y方向に指定後、柱はx、梁はz方向に2組以上の指定
    int rest = 0;
    int dir  = 0;
    int delt = 0;
    int body = 0;
    int reverse = 0;
    if(opt1 == 0)
    {
        //柱加力
        rest = 1;//z方向
        dir  = 2;
        delt = (geo[1].boundary[2] - geo[1].boundary[1]) * pp[1];
        body = 3;
        reverse = geo[0].boundary[6] * pp[0];
    }
    else if(opt1 == 1)
    {
        //梁加力
        rest = 100; //x方向
        dir  = 0;
        delt = (geo[1].boundary[2] - geo[1].boundary[0]) * pp[1];
        body = 4;
        reverse = (geo[2].boundary[5] + 2)* pp[2];
    }
    else
    {
        printf("error\n");
        return;
    }
    
    fprintf(f, "\n----\n");
    int node = startNode;
    print_rest(f, node, node + delt, pp[1], rest, pp[dir], geo[dir].boundary[body] - geo[dir].boundary[2] + opt1);
    node += reverse;
    print_rest(f, node, node + delt, pp[1], rest, pp[dir], geo[dir].boundary[body] - geo[dir].boundary[2] + opt1);
}

void set_roller(FILE *f, int startNode, struct loadNode load, const Geometry geo[], int pp[], int opt1, int span)
{
    //y方向に指定後、柱加力はx、梁加力はz方向に
    int dir = 0;
    int sub = 0;
    int delt = 0; //y方向にずらす
    int body = 0;
    int reverse = 0;
    if(opt1 == 0)
    {
        //柱加力
        dir = 0; //x方向
        sub = 1;
        delt = (geo[1].boundary[2] - geo[1].boundary[0]) * pp[1];
        body = 4;
        reverse = (geo[2].boundary[5] + 2) * pp[2];
    }
    else if(opt1 == 1)
    {
        //梁加力
        dir = 2; //z方向
        sub = 3;
        delt = (geo[1].boundary[2] - geo[1].boundary[1]) * pp[1];
        body = 3;
        reverse = geo[0].boundary[6] * pp[0];
    }
    else
    {
        printf("error\n");
        return;
    }
    int loadNode[2];
    if(load.node1 < load.node2)
    {
        loadNode[0] = load.node1;
        loadNode[1] = load.node2;
    }else{
        loadNode[0] = load.node2;
        loadNode[1] = load.node1;
    }
    for(int j = 0; j <= 1; j++)
    {
        for(int i = geo[dir].boundary[2]; i <= geo[dir].boundary[body]; i++)
        {
            int node = startNode + (i - geo[dir].boundary[2]) * pp[dir] + j * reverse;
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
    print_rest(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] + 1) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[2] - geo[2].boundary[0]);
    column = columnStart + geo[1].boundary[2] * columnPp[1] + (geo[2].boundary[3] + 2) * columnPp[2];
    print_rest(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] + 1) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[5] - geo[2].boundary[3]);
    print_rest(f, jointStart + geo[1].boundary[2] * columnPp[1], jointStart + geo[1].boundary[2] * columnPp[1] + (geo[0].boundary[4] - geo[0].boundary[2]) * columnPp[0], columnPp[0], 10, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
    //梁
    print_rest(f, beam, beam + (geo[0].boundary[2] - 1) * beamPp[0], beamPp[0], 10, beamPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
    beam += (geo[0].boundary[4] + 1) * beamPp[0];
    print_rest(f, beam, beam + (geo[0].boundary[6] - geo[0].boundary[4] - 1) * beamPp[0], beamPp[0], 10, beamPp[2], geo[2].boundary[3] - geo[2].boundary[2]);
}

void add_typh(FILE *f, const Geometry geo[], const ModelSize rcs, int columnElm, int columnPp[])
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
    for(int j = 0; j <= 4; j += 4)
    {
        for(int i = geo[2].boundary[j]; i < geo[2].boundary[j + 1]; i++)
        {
            column = columnElm + i * columnPp[2];
            print_etyp(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 5, columnPp[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        }
    }
    //かぶり
    for(int j = 1; j <= 3; j += 2)
    {
        for(int i = 0; i < min[1]; i++)
        {
            column = columnElm + i * columnPp[1] + geo[2].boundary[j] * columnPp[2];
            print_etyp(f, column, column + (geo[0].boundary[4] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 2, columnPp[2], geo[2].boundary[j + 1] - geo[2].boundary[j] - 1);
        }
        for(int i = 0; i < geo[0].boundary[4] - geo[0].boundary[2]; i++)
        {
            if(i < min[0] || i >= max[0])
            {
                column = columnElm + i * columnPp[0] + min[1] * columnPp[1] + geo[2].boundary[j] * columnPp[2];
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
        column = columnElm + i * columnPp[1] + geo[2].boundary[2] * columnPp[2];
        print_etyp(f, column, column + (min[0] - 1) * columnPp[0], columnPp[0], 4, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        column += max[0] * columnPp[0];
        print_etyp(f, column, column + (geo[0].boundary[4] - max[0] - geo[0].boundary[2] - 1) * columnPp[0], columnPp[0], 4, columnPp[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    }
}

column count_column_Numbers(const Geometry geo[], const ModelSize *rcs)
{
    column numbers;
    //柱六面体節点数
    numbers.node.hexa  = (geo[0].boundary[4] - geo[0].boundary[2] + 2)
                       * (geo[1].boundary[2] - geo[1].boundary[0] + 1)
                       * (geo[2].boundary[5] - geo[2].boundary[0] + 3);
    //線材節点数
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs->rebar[i].cordi[0] < 0)
        {
            numbers.node.fiber = i * (geo[2].boundary[4] - geo[2].boundary[1] + 3);
            break;
        }
    }
    //柱六面体要素数
    numbers.elm.hexa = (geo[0].boundary[4] - geo[0].boundary[2])
                     * (geo[1].boundary[2] - geo[1].boundary[0])
                     * (geo[2].boundary[5] - geo[2].boundary[0]);
    //線材要素数
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs->rebar[i].cordi[0] < 0)
        {
            numbers.elm.fiber = i * (geo[2].boundary[4] - geo[2].boundary[1]);
            break;
        }
    }
    //ライン要素数
    numbers.elm.line = numbers.elm.fiber;

    return numbers;
}

joint count_joint_numbers(const Geometry geo[])
{
    joint numbers;
    //四辺形節点数 - 穴あき部分の節点もカウント
    numbers.node.quad  = (geo[0].boundary[4] - geo[0].boundary[2] + 1)
                       * (geo[1].boundary[2] - geo[1].boundary[0] + 1)
                       * (geo[2].boundary[3] - geo[2].boundary[2] + 1);
    //四辺形要素数 - フィルム要素と数が違うが接合部の数え方は共通なので同じとする
    numbers.elm.quad   = (geo[0].boundary[4] - geo[0].boundary[2] + 4)
                       * (geo[1].boundary[2] - geo[1].boundary[0] + 2)
                       * (geo[2].boundary[3] - geo[2].boundary[2] + 4);
    //フィルム要素数
    numbers.elm.film  = numbers.elm.quad;

    return numbers;
}

beam count_beam_numbers(const Geometry geo[])
{
    beam numbers;
    //梁節点数
    numbers.node = (geo[0].boundary[6] - geo[0].boundary[4] + geo[0].boundary[2])
                 * (geo[1].boundary[2] - geo[1].boundary[1] + 1)
                 * (geo[2].boundary[3] - geo[2].boundary[2] + 1);
    //梁要素数
    numbers.elm = (geo[0].boundary[6] - geo[0].boundary[4] + geo[0].boundary[2])
                * (geo[1].boundary[2] - geo[1].boundary[1] + 1)
                * (geo[2].boundary[3] - geo[2].boundary[2] + 2);

    return numbers;
}

column set_column_heads(const Geometry geo[], const ModelSize *rcs, const column *numbers)
{
    column head;
    //六面体要素
    head.node.hexa = FIRST_NODE;
    head.elm.hexa  = FIRST_ELM;
    //線材要素
    head.node.fiber =  next_index(numbers->node.hexa + head.node.hexa - 1);
    head.elm.fiber  =  next_index(numbers->elm.hexa + head.elm.hexa - 1);
    //ライン要素
    head.elm.line = next_index(head.elm.fiber + numbers->elm.fiber - 1);
    
    return head;
}

joint set_joint_heads(const column *columnHeads, const column *columnNumbers, const joint *jointNumbers, int columnPp, const ModelSize *rcs, const Geometry geo[])
{
    joint heads;
    //四辺形要素
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs->rebar[i].cordi[0] < 0)
        {
            heads.node.quad = next_index(columnHeads->node.fiber + (geo[2].boundary[4] - geo[2].boundary[1] + 2) * columnPp + i);
            break;
        }
    }

    heads.elm.quad  = next_index(columnHeads->elm.line - 1 + columnNumbers->elm.line);
    //フィルム要素
    heads.elm.film  = next_index(heads.elm.quad - 1 + jointNumbers->elm.quad);
    return heads;
}

beam set_beam_heads(const joint *jointNumbers, const joint *jointHeads)
{
    beam heads;
    //節点
    heads.node = next_index(jointHeads->node.quad - 1 + jointNumbers->node.quad);
    //要素
    heads.elm = next_index(jointHeads->elm.film - 1 + jointNumbers->elm.film);
    return heads;
}

void modeling_rcs(const char *inputFileName, int opt1, const char *stepDispFile)
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

    LoadNode  roller;
    LoadNode  pin;

    printf("\n[START]\n");

    /*ファイル読み込み*/
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
    const column columnNumbers = count_column_Numbers(modelGeometry, &rcs);
    const joint  jointNumbers  = count_joint_numbers(modelGeometry);
    const beam   beamNumbers   = count_beam_numbers(modelGeometry);

    printf("non\n----\n");
    
    //試験体情報表示
    console_model_sizes(rcs);

    //メッシュ情報表示
    console_mesh(modelGeometry);
    printf("\n");

    //境界線情報表示
    console_boundary(modelGeometry);

    //インクリメント
    Increment columnPp;
    get_column_increment(&columnPp, modelGeometry);

    Increment jointPp;
    get_joint_increment(&columnPp, &jointPp, modelGeometry);

    Increment beamPp;
    get_beam_increment(&beamPp, modelGeometry);

    console_increment(&columnPp);
    console_increment(&jointPp);
    console_increment(&beamPp);

    //各部分の最初の番号
    const column columnHeads = set_column_heads(modelGeometry, &rcs, &columnNumbers);
    const joint  jointHeads  = set_joint_heads(&columnHeads, &columnNumbers, &jointNumbers, columnPp.node[2], &rcs, modelGeometry);
    const beam   beamHeads   = set_beam_heads(&jointNumbers, &jointHeads);

    console_numbers(&columnNumbers, &jointNumbers, &beamNumbers);

    StepDisp stepDisp[STEP_DISP_MAX];
    if(stepDispFile != NULL)
    {
        read_loading_step(stepDispFile, stepDisp);
        console_step_disp(stepDisp);
    }

    get_load_node(&pin, &roller, columnHeads.node.hexa, beamHeads.node, columnPp.node, beamPp.node, modelGeometry, rcs.column.span, opt1);

    //node
    /*
    FILE *fnode = fopen("node.txt", "w");
    if(fnode != NULL)
    {
        fprintf(fnode, "--\nparts   node      incX     Y     Z\n");
        fprintf(fnode, "column %5d  %5d %5d %5d\n", columnHexa.node, columnPp.node[0], columnPp.node[1], columnPp.node[2]);
        fprintf(fnode, "joint  %5d  %5d %5d %5d\n", jointQuad.node, columnPp.node[0], columnPp.node[1], columnPp.node[2]);
        fprintf(fnode, "beam   %5d  %5d %5d %5d\n", beam.node, beamPp.node[0], beamPp.node[1], beamPp.node[2]);
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
    */
    //ファイルオープン
    FILE *fout = fopen(OUT_FILE_NAME,"w");
    if(fout == NULL)
    {
        printf("[ERROR] out.ffi cant open.\n");
        return ;
    }

    //解析制御データ
    if(stepDispFile == NULL)
    {
        print_head_template(fout, roller, opt1, 11);    
    }
    else
    {
        for(int i = 0; i < STEP_DISP_MAX; i++)
        {
            if(stepDisp[i].step < 0)
            {
                print_head_template(fout, roller, opt1, stepDisp[--i].step);
                break;
            }  
        }
    }

    //柱六面体要素
    add_column_hexa(fout, &columnHeads, columnPp, modelGeometry);
    add_typh(fout, modelGeometry, rcs, columnHeads.elm.hexa, columnPp.elm);

    //柱線材要素
    add_reber(fout, &columnPp, &rcs, modelGeometry, &columnHeads);

    //接合部四辺形要素
    add_joint_quad(fout, &columnHeads, &jointHeads, modelGeometry, columnPp, rcs);

    //梁六面体要素
    hexa_beam(fout, &beamHeads, beamPp, modelGeometry);

    //梁四辺形要素
    quad_beam(fout, &beamHeads, columnPp, beamPp, modelGeometry, &jointHeads);
    
    //柱コンクリート節点結合
    merge_nodes(fout, columnHeads.node.hexa, modelGeometry, rcs, columnPp.node);

    //柱加力or梁加力
    if(opt1 == 0)
    {
        //柱加力
        set_pin(fout, beamHeads.node, modelGeometry, beamPp.node, opt1);
        set_roller(fout, columnHeads.node.hexa, roller, modelGeometry, columnPp.node, opt1, rcs.beam.span);
    }else if(opt1 == 1)//梁加力
    {
        set_pin(fout, columnHeads.node.hexa, modelGeometry, columnPp.node, opt1);
        set_roller(fout, beamHeads.node, roller, modelGeometry, beamPp.node, opt1, rcs.column.span);
    }

    cut_surface(fout, modelGeometry, columnHeads.node.hexa, jointHeads.node.quad, beamHeads.node, columnPp.node, beamPp.node);

    print_type_mat(fout);
    print_column_axial_force(fout, columnHeads.elm.hexa, modelGeometry, rcs.column.Fc, columnPp.elm);

    if(stepDispFile == NULL)
    {
        print_test_step(fout, roller, opt1);
    }
    else
    {
        print_loading_step(fout, roller, opt1, stepDisp);
    }

    fclose(fout);

    printf("\n[END]\n");

}

void out_template()
{
    FILE *f = fopen(TEMPLATE_FILE_NAME, "r");
    if(f == NULL)
    {
        f = fopen(TEMPLATE_FILE_NAME, "w");
        if(f != NULL)
        {
            fprintf(f,
                "Column : Span() Width() Depth() X_Center() Y_Center()\n"
                "Beam   : Span() Width() Depth() Y_Center() Z_Center()\n"
                "xBeam  : Width()\n"
                "Rebar 1: X() Y()\n"
                "Rebar 2: X() Y()\n"
                "Rebar 3: X() Y()\n"
                "--\n"
                "X_Mesh_Sizes(1,2,3)\n"
                "Y_Mesh_Sizes(1,2,3)\n"
                "Z_Mesh_Sizes(1,2,3)\n"
                "END\n");
            fclose(f);
        }
        else
        {
            printf("[ERROR] file cant open\n");
            return ;
        }
    }
    else
    {
        printf(TEMPLATE_FILE_NAME " already exist\n");
        fclose(f);
    }
    return ;
}

void out_loading_template()
{
    FILE *f = fopen(LOADING_TEMPLATE_FILE_NAME, "r");
    if(f == NULL)
    {
        f = fopen(LOADING_TEMPLATE_FILE_NAME, "w");
        if(f != NULL)
        {
            fprintf(f,
                "step(2,3,4)\n"
                "disp(5,-5,10)\n"
                "END\n");
            fclose(f);
        }
        else
        {
            printf("[ERROR] file cant open\n");
            return ;
        }
    }
    else
    {
        printf(LOADING_TEMPLATE_FILE_NAME " already exist\n");
        fclose(f);
    }
    return ;
}

void show_usage()
{
    printf(
        "options\n"
        "-t : out template\n"
        "-l : out loading template\n"
        "-b : beam -> roller  column -> pin\n"
        "-s : write loading step\n"
        "-f : full model \n"
    );
}

void console_version()
{
    const char *date = "2024/05/22";
    printf("last updated -> %s\n", date);
}

int main(int argc, char *argv[])
{
    /*
        command [options] [arguments]
    */
    int         optchar;
    int         optb = 0;
    const char *inputFile = NULL;
    const char *stepDispFile = NULL;

    if(argc < 2)
    {
        show_usage();
        return 1;
    }
    while((optchar = getopt(argc, argv, OPTCHAR)) != -1)
    {
        switch(optchar){
            case 't': //テンプレ出力
                printf("option t\n");
                out_template();
                return 0;
            case 'l': //laoding step出力
                printf("option l\n");
                out_loading_template();
                return 0;
            case 'b': //beam 梁加力
                optb = 1;
                printf("option b\n");
                break;
            case 's':
                stepDispFile = optarg;
                printf("option s\n"
                       "Loading step file is \"%s\"\n", stepDispFile);
                break;
            case 'v':
                printf("option v\n");
                console_version();
                return 0;
            default:
                show_usage();
                return 0;
        }
    }
    if(optind < argc){
        inputFile = argv[optind];
    }else{
        fprintf(stderr, "No input file specified\n");
        return 1;
    }
    if(inputFile != NULL){
        printf("Input file is \"%s\"\n", inputFile);
        modeling_rcs(inputFile, optb, stepDispFile);
    }
    return 0;
}