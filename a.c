#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define INPUT_FILE_NAME "input.txt"
#define OUT_FILE_NAME   "out.ffi"
#define MESH_ARRAY_MAX 64 //注意が必要
#define REBAR_MAX      16 
#define BOUNDARY_MAX   8 
#define XYZ            3    

struct components
{
    double  span;
    double  width;
    double  depth;
    double  center[XYZ];
};

struct Rebar 
{
    double x;
    double y;
};

struct modelsize
{
    struct components column;
    struct components beam;
    struct components xbeam;
    struct Rebar      rebar[REBAR_MAX]; //{{主筋の番号}{0->x, 1以降->y}}
};

struct Mesh
{
    int    number; //メッシュ配列の要素数
    double length  [MESH_ARRAY_MAX];
};

struct geometry
{
    struct Mesh mesh;
    int         boundary[BOUNDARY_MAX];
};

struct nodeElm 
{
    int node;
    int elm;
};

struct Point
{
    int x;
    int y;
    int z;
};

struct startEnd
{
    struct Point start;
    struct Point end;
};

/*戻り値//要素数*/
int load_mesh(FILE *f, double meshLen[], int dir)
{
    int ch;
    int count = 0;

    ch = fgetc(f);
    if (ch != dir) {
        printf("[ERROR] dir\n");
        return -1;
    }
    
    // "_mesh_sizes:" の読み飛ばし
    fscanf(f, "_Mesh_Sizes:");

    // 数値の読み取り
    while (1) 
    {
        double value;
        int result = fscanf(f, "%lf", &value); // カンマを含めないで読み込む
        if (result == EOF) {
            // ファイルの終わりに達した場合はループを終了
            break;
        } else if (result == 0) {
            // 数値の読み取りに失敗した場合は終了
            printf("[ERROR] load mesh\n");
            return -1;
        }
        
        meshLen[count++] = value;

        // カンマを読み飛ばす
        ch = fgetc(f);
        if (ch == '\n') 
        {
            break;
        }
        else if(ch != ',')
        {
            ungetc(ch, f); // カンマでない文字はファイルに戻す
        }
    }
    return count;
}

struct modelsize Model_Size_Load(FILE *f)
{
    struct modelsize temp = {{0}};
    int index = 0;

    //柱、梁情報
    fscanf(f, "Column : Span(%lf) Width(%lf) Depth(%lf) X_Center(%lf) Y_Center(%lf)",
        &temp.column.span, &temp.column.width, &temp.column.depth, &temp.column.center[0], &temp.column.center[1]);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "Beam   : Span(%lf) Width(%lf) Depth(%lf) Y_Center(%lf) Z_Center(%lf)",
        &temp.beam.span, &temp.beam.width, &temp.beam.depth, &temp.beam.center[1], &temp.beam.center[2]);
    while((fgetc(f)) != '\n'){}
    fscanf(f, "xBeam  : Width(%lf)",&temp.xbeam.width);
    while((fgetc(f)) != '\n'){}

    //主筋情報
    fscanf(f, "Rebar %d:", &index);
    while(index <= REBAR_MAX)
    {
        fscanf(f, "Rebar %d:", &index);
        int result = fscanf(f, " X=%lf Y=%lf", &temp.rebar[index - 1].x, &temp.rebar[index - 1].y);
        if (result == EOF) {
            // ファイルの終わりに達した場合はループを終了
            break;
        } else if (result == 0) {
            // 数値の読み取りに失敗した場合はエラーを表示して終了
            printf("[ERROR] load mesh\n");
            temp.rebar[index].x = -999;
            temp.rebar[index].y = -999;
            break;
        }
        temp.rebar[index - 1].x += (temp.beam.span - temp.column.depth) / 2;
        while((fgetc(f)) != '\n'){}
    }
    return temp;
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

int coordinate_to_point(double cordi, double array[])
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

double point_to_coordinate(int point, double array[])
{
    double cordi = 0;
    for(int i = 0; i < point; i++)
    {
        cordi += array[i];
    }
    return cordi;
}

int count_consecutive(int start, int end, double array[])
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

void get_increment()
{

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

void console_model_sizes(struct modelsize rcs)
{
    printf("\n------------------[ MODEL ]------------------\n\n");
    printf("          span     width    depth    center\n");
    printf("column   | %6.1f | %6.1f | %6.1f |x %6.1f |y %6.1f \n", rcs.column.span, rcs.column.width, rcs.column.depth, rcs.column.center[0], rcs.column.center[1]);
    printf("beam     | %6.1f | %6.1f | %6.1f |y %6.1f |z %6.1f \n", rcs.beam.span, rcs.beam.width, rcs.beam.depth, rcs.beam.center[1], rcs.beam.center[2]);
    printf("xbeam    |        | %6.1f |        |y        |z    \n\n", rcs.xbeam.width);
    
    printf("reber");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf("%7d", i + 1);
    }
    printf("\nX    ");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf("%7.1f", rcs.rebar[i].x);
    }
    printf("\nY    ");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf("%7.1f", rcs.rebar[i].y);
    }
    printf("\n");

}

void console_mesh(struct geometry modelGeometry[], int dir[])
{
    printf("\n------------------[ MESH ]------------------\n");
    for(int j = 0; j < 3; j++)
    {
        printf("\n%c mesh count --> %d\n", dir[j], modelGeometry[j].mesh.number);
        printf("  i :");
        for(int i = 0; i < modelGeometry[j].mesh.number; i++)
        {
            printf(" %5d", i);
        }
        printf("\nlen :");
        for(int i = 0; i < modelGeometry[j].mesh.number; i++)
        {
            printf(" %5.1f", modelGeometry[j].mesh.length[i]);
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

void console_increment(struct nodeElm pp[])
{
    printf("\n------------------[ INCREMENT ]------------------\n");
    printf("         X     Y     Z\n");
    printf("node %5d %5d %5d\n", pp[0].node, pp[1].node, pp[2].node);
    printf("elm  %5d %5d %5d\n", pp[0].elm, pp[1].elm, pp[2].elm);
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
void print_head_template(FILE *f)
{
    fprintf(f,
    "-------------------< FINAL version 11  Input data >---------------------\n"
    "TITL :\n"
    "EXEC :STEP (    1)-->(     )  ELASTIC=( ) CHECK=(1) POST=(1) RESTART=( )\n"
    "LIST :ECHO=(0)  MODEL=(1)  RESULTS=(1)  MESSAGE=(2)  WARNING=(2)  (0:NO)\n"
    "FILE :CONV=(2)  GRAPH=(2)  MONITOR=(2)  HISTORY=(1)  ELEMENT=(0)  (0:NO)\n"
    "DISP :DISPLACEMENT MONITOR NODE NO.(     )  DIR=( )    FACTOR=\n"
    "LOAD :APPLIED LOAD MONITOR NODE NO.(     )  DIR=( )    FACTOR=\n"
    "UNIT :STRESS=(3) (1:kgf/cm**2  2:tf/m**2  3:N/mm**2=MPa)\n\n");
}

void print_tail_template(FILE *f)
{
    fprintf(f,
    "\n----+----\n"
    "TYPH :(  1)  MATS(  1)  AXIS(   )\n"
    "TYPH :(  2)  MATC(  1)  AXIS(   )\n"
    "TYPH :(  3)  MATC(  1)  AXIS(   )\n"
    "TYPB :(  1)  MATS(  2)  AXIS(   )  COEF=        D1=1       D2=        :\n"
    "TYPQ :(  1)  MATS(  1)  AXIS(   )  THICKNESS=1       P-STRAIN=(0) (0:NO)\n"
    "TYPL :(  1)  MATJ(  1)  AXIS(  1)  THICKNESS=1        Z=(1) (1:N  2:S)\n"
    "\n----+----\n"
    "MATS :(  1)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATS :(  2)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATC :(  1)  EC=2.05   (E+4) PR=0.2   FC=30     FT=3     ALP=      (E-5)\n"
    "MATJ :(  1)  TYPE=(4) (1:CRACK  2:BOND  3:GENERIC  4:RIGID  5:DASHPOT)\n"
    "\n----+----\n"
    "AXIS :(  1)  TYPE=(1) (1:GLOBAL 2:ELEMENT 3:INPUT 4:CYLINDER 5:SPHERE)\n"
    "\n----+----\n"
    "STEP :UP TO NO.(    1)   MAXIMUM LOAD INCREMENT=         CREEP=(0)(0:NO)\n"
    "  FN :NODE  S(    1)-E(     )-I(     )    FORCE=10       DIR(1)\n"
    " OUT :STEP  S(    1)-E(     )-I(     ) LEVEL=(3) (1:RESULT 2:POST 3:1+2)\n"
    "\nEND\n");
}

/*NODEデータ書き込み*/
void print_NODE(FILE *f, int nodeIndex, double coordinates[]) 
{
    fprintf(f, "NODE :(%5d)  X=%-10.1fY=%-10.1fZ=%-10.1fRC=(000000)\n", nodeIndex, coordinates[0], coordinates[1], coordinates[2]);
}

/*COPY:NODEデータ書き込み*/
int print_COPYNODE(FILE *f, int startIndex, int endIndex, int interval, double meshLen, int increment, int set, int dir) 
{
    int index = startIndex;
    char end[6] = " ";
    char _interval[6] = " ";

    if (endIndex != 0)
    {
        sprintf(end, "%d", endIndex);
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
            case 0  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DX=%-9.1lfINC(%5d)-SET(%4d)\n", index, end, _interval, meshLen, increment, set); break;
            case 1  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DY=%-9.1lfINC(%5d)-SET(%4d)\n", index, end, _interval, meshLen, increment, set); break;
            case 2  : fprintf(f, "COPY :NODE  S(%5d)-E(%5s)-I(%5s)  DZ=%-9.1lfINC(%5d)-SET(%4d)\n", index, end, _interval, meshLen, increment, set); break;
            default : printf("[ERROR] CopyNode\n");
        }
    }
    if(endIndex != 0)
    {
        return   endIndex + increment * set;
    }else{
        return startIndex + increment * set;
    }
}

void print_BEAM(FILE *f, int elmIndex, int startNode, int endNode, int TYPB)
{
    fprintf(f, "BEAM :(%5d)(%5d:%5d) TYPB(%3d)  Y-NODE(     )\n", elmIndex, startNode, endNode, TYPB);
}

void print_QUAD(FILE *f, int elmIndex, int startNode, int node_pp[], int dir1, int dir2, int TYPQ)
{
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", elmIndex, startNode, startNode + node_pp[dir1], startNode + node_pp[dir1] + node_pp[dir2], startNode + node_pp[dir2], TYPQ);
}

void print_HEXA(FILE *f, int EleIndex, int Node_S, int node_pp[], int TYPH)
{
    fprintf(f, "HEXA :(%5d)(%5d:%5d:%5d:%5d:%5d:%5d:%5d:%5d) TYPH(%3d)\n", EleIndex, Node_S, Node_S + node_pp[0], Node_S + node_pp[0] + node_pp[1], Node_S + node_pp[1], Node_S + node_pp[2], Node_S + node_pp[0] + node_pp[2], Node_S + node_pp[0] + node_pp[1] + node_pp[2], Node_S + node_pp[1] + node_pp[2], TYPH);
}

void print_LINE(FILE *f, int elmIndex, int nodeIndex1, int nodeIndex3, int pp)
{
    fprintf(f, "LINE :(%5d)(%5d:%5d:%5d:%5d) TYPL(  1)\n", elmIndex, nodeIndex1, nodeIndex1 + pp, nodeIndex3, nodeIndex3 + pp);
}

void print_FILM(FILE *f, int elmIndex, int face1, int face2, int nodePp[3], int dir1, int dir2, int typf)
{
    fprintf(f, "FILM :(%5d)(%5d:%5d:%5d:%5d:%5d:%5d:%5d:%5d) TYPF(%3d))\n", elmIndex, face1, face1 + nodePp[dir1], face1 + nodePp[dir1] + nodePp[dir2], face1 + nodePp[dir2], face2, face2 + nodePp[dir1], face2 + nodePp[dir1] + nodePp[dir2], face2 + nodePp[dir2], typf);
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

/*--要素生成-----------------------------------------------*/
void generate_beam(FILE *f, struct nodeElm pp[], double coordinates[], struct startEnd point, struct nodeElm startIndex, double meshLen[], int dir)
{
    const int elmPPz = 1;
    int cnt;
    int index = startIndex.node;
    //節点定義
    print_NODE(f, index, coordinates);
    //節点コピー
    for(int i = point.start.z; i < point.end.z; i += cnt)
    {
        cnt = count_consecutive(i, point.end.z, meshLen);
        print_COPYNODE(f, index, 0, 0, meshLen[i], pp[2].node, cnt, dir);
        index += cnt * pp[2].node;
    }
    //要素定義
    index = startIndex.elm;
    print_BEAM(f, index, startIndex.node, startIndex.node + pp[2].node, 1);
    //要素コピー
    print_COPYELM(f, index, 0, 0, elmPPz, pp[2].node, point.end.z - point.start.z - 1);
}

struct nodeElm generate_hexa(FILE *f, struct nodeElm startIndex, struct nodeElm pp[], struct startEnd point, double coordinates[], struct geometry geo[], int TYPH)
{
    int cnt;
    int delt;
    struct nodeElm index;
    struct nodeElm returnIndex;
    index.node = startIndex.node;
    index.elm  = startIndex.elm;
    //節点定義
    print_NODE(f, index.node, coordinates);
    //節点コピー
    //x
    for(int i = point.start.x; i < point.end.x; i += cnt)
    {
        cnt = count_consecutive(i, point.end.x, geo[0].mesh.length);
        returnIndex.node = print_COPYNODE(f, index.node, 0, 0, geo[0].mesh.length[i], pp[0].node, cnt, 0);
        index.node += cnt * pp[0].node;
    }
    //y
    delt = index.node - startIndex.node;
    index.node = startIndex.node;
    for(int i = point.start.y; i < point.end.y; i += cnt)
    {
        cnt = count_consecutive(i, point.end.y, geo[1].mesh.length);
        returnIndex.node = print_COPYNODE(f, index.node, index.node + delt, pp[0].node, geo[1].mesh.length[i], pp[1].node, cnt, 1);
        index.node += cnt * pp[1].node;
    }
    //z
    for(int j = 0; j <= point.end.y - point.start.y;j++)
    {
        index.node = startIndex.node;
        for(int i = point.start.z; i < point.end.z; i += cnt)
        {
            cnt = count_consecutive(i, point.end.z, geo[2].mesh.length);
            returnIndex.node = print_COPYNODE(f, index.node + pp[1].node * j, index.node + delt + pp[1].node * j, pp[0].node, geo[2].mesh.length[i], pp[2].node, cnt, 2);
            index.node += cnt * pp[2].node;
        }
    }
    fprintf(f, "\n");
    //要素定義
    int ppNode[3];
    ppNode[0] = pp[0].node;
    ppNode[1] = pp[1].node;
    ppNode[2] = pp[2].node;
    print_HEXA(f, startIndex.elm, startIndex.node, ppNode, TYPH);
    //要素コピー
    //x
    returnIndex.elm = print_COPYELM(f, index.elm, 0, 0, pp[0].elm, pp[0].node, point.end.x - point.start.x - 1);
    //y
    delt = (point.end.x - point.start.x - 1) * pp[0].elm;
    returnIndex.elm = print_COPYELM(f, index.elm, index.elm + delt, pp[0].elm, pp[1].elm, pp[1].node, point.end.y - point.start.y - 1);
    //z
    for(int i = 0; i < point.end.y - point.start.y; i++)
    {
        returnIndex.elm = print_COPYELM(f, index.elm + pp[1].elm * i, index.elm + delt + pp[1].elm * i, pp[0].elm, pp[2].elm, pp[2].node, point.end.z - point.start.z - 1);
    }
    return returnIndex;
}

void generate_line(FILE *f, int elmIndex, int hexaNodeIndex, int beamNodeIndex, int node_pp_z, int set)
{
    const int elmPPz = 1;
    int index = elmIndex;
    print_LINE(f, index, hexaNodeIndex, beamNodeIndex, node_pp_z);
    print_COPYELM(f, elmIndex, 0, 0, elmPPz, node_pp_z, set);
}

struct nodeElm add_reber(FILE *f, struct nodeElm pp[], struct modelsize rcs, struct geometry modelGeometry[], struct nodeElm startIndex)
{
    const int dir = 2; //z方向
    struct nodeElm index;
    struct startEnd point;
    double coordinates[3];

    int concreteStartNode;

    int elmNum = modelGeometry[2].boundary[4] - modelGeometry[2].boundary[1];
    for(int i = 0; i < REBAR_MAX; i++)
    {
        if(rcs.rebar[i].x < 0)
        {
            elmNum *= i;
            elmNum = next_index(elmNum) - 1;
            break;
        }
    }

    fprintf(f, "\n----REBAR----");
    index.elm  = startIndex.elm;
    for(int j = 0; j < REBAR_MAX; j++)
    {
        if(rcs.rebar[j].x < 0)
        {
            break;
        }
        index.node = startIndex.node + j;
        int xIndex = coordinate_to_point(rcs.rebar[j].x, modelGeometry[0].mesh.length) - modelGeometry[0].boundary[2];
        int yIndex = coordinate_to_point(rcs.rebar[j].y, modelGeometry[1].mesh.length) - modelGeometry[1].boundary[0];
        if(coordinate_to_point(rcs.rebar[j].x, modelGeometry[0].mesh.length) > modelGeometry[0].boundary[3])
        {
            xIndex++;
        }
        int concreteAddNumber = pp[0].node * xIndex + pp[1].node * yIndex;

        fprintf(f, "\n----\n");
        coordinates[0] = rcs.rebar[j].x;
        coordinates[1] = rcs.rebar[j].y;
        for(int i = 1; i < 4; i++)
        {

            if(rcs.rebar[i].x < 0)
            {
                break;
            }
            point.start.z = modelGeometry[dir].boundary[i];
            point.end.z   = modelGeometry[dir].boundary[i + 1];
            coordinates[dir] = point_to_coordinate(modelGeometry[dir].boundary[i], modelGeometry[dir].mesh.length);
            generate_beam(f, pp, coordinates, point, index, modelGeometry[dir].mesh.length, dir);


            concreteStartNode = (1 + (modelGeometry[2].boundary[i] + i) * pp[2].node) + concreteAddNumber;
            generate_line(f, index.elm + elmNum, concreteStartNode, index.node, pp[2].node, modelGeometry[dir].boundary[i + 1] - modelGeometry[dir].boundary[i] - 1);
            concreteStartNode += pp[dir].node   * (modelGeometry[dir].boundary[i + 1] - modelGeometry[dir].boundary[i] + 1);
            index.node += pp[dir].node   * (modelGeometry[dir].boundary[i + 1] - modelGeometry[dir].boundary[i] + 1);
            index.elm  += 1/*1づつ増える*/* (modelGeometry[dir].boundary[i + 1] - modelGeometry[dir].boundary[i]);
        }

    }
    index.elm  += elmNum;
    return index;
}

struct nodeElm add_concrete(FILE *f, struct nodeElm startIndex, struct nodeElm pp[], struct geometry modelGeometry[])
{
    int typh[5] = {1, 2, 3, 2, 1};
    double coordinates[3];
    struct nodeElm index;
    struct nodeElm returnIndex;
    struct startEnd point;
    index.node = startIndex.node;
    index.elm = startIndex.elm;

    for(int j = 0; j < 5; j++)
    {
        fprintf(f, "\n----TYPH %1d----", typh[j]);
        for(int i = 0; i < 2; i++)
        {
            fprintf(f, "\n----\n");
            point.start.x = modelGeometry[0].boundary[2 + i];
            point.start.y = modelGeometry[1].boundary[0];
            point.start.z = modelGeometry[2].boundary[0 + j];
            point.end.x   = modelGeometry[0].boundary[3 + i];
            point.end.y   = modelGeometry[1].boundary[2];
            point.end.z   = modelGeometry[2].boundary[1 + j];
            
            coordinates[0] = point_to_coordinate(modelGeometry[0].boundary[2 + i], modelGeometry[0].mesh.length);
            coordinates[1] = point_to_coordinate(modelGeometry[1].boundary[0], modelGeometry[1].mesh.length);
            coordinates[2] = point_to_coordinate(modelGeometry[2].boundary[0 + j], modelGeometry[2].mesh.length);
            returnIndex = generate_hexa(f, index, pp, point, coordinates, modelGeometry, typh[j]);
            index.node += (modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2] + 1) * pp[0].node;
            index.elm  += (modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2])     * pp[0].elm;
        }
        index.node = startIndex.node + (modelGeometry[2].boundary[j + 1] + j + 1) * pp[2].node;
        index.elm  = startIndex.elm  +  modelGeometry[2].boundary[j + 1]          * pp[2].elm;
    }
    return returnIndex;
}

void quad_collumn(FILE *f, struct nodeElm startIndex, struct geometry modelGeometry[], struct nodeElm pp[], struct modelsize rcs)
{
    int cnt = 0;
    double cordi[XYZ];
    struct nodeElm index;
    int pp_node[3];
    for(int i = 0; i < 3; i++)
    {
        pp_node[i] = pp[i].node;
    }
    int pp_elm[3];
    pp_elm[0] = 1;
    pp_elm[1] = pp[1].elm + 3;
    pp_elm[2] = pp_elm[1] * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 2);

    index = startIndex;
    int startIndexNode = startIndex.node;
    fprintf(f, "\n----COLUMN QUAD----\n");
    //x直交面
    fprintf(f, "\n----x\n");
    for(int j = 2; j < 5; j++)
    {
        index.node = startIndexNode;
        cordi[0] = point_to_coordinate(modelGeometry[0].boundary[j], modelGeometry[0].mesh.length);
        cordi[1] = point_to_coordinate(modelGeometry[1].boundary[0], modelGeometry[1].mesh.length);
        cordi[2] = point_to_coordinate(modelGeometry[2].boundary[2], modelGeometry[2].mesh.length);
        print_NODE(f, index.node, cordi);
        for(int i = modelGeometry[1].boundary[0]; i < modelGeometry[1].boundary[2]; i += cnt)
        {
            cnt = count_consecutive(i, modelGeometry[1].boundary[2], modelGeometry[1].mesh.length);
            print_COPYNODE(f, index.node, 0, 0, modelGeometry[1].mesh.length[i], pp[1].node, cnt, 1);
            index.node += cnt * pp[1].node;
        }
        int delt = index.node - startIndexNode;
        index.node = startIndexNode;
        for(int i = modelGeometry[2].boundary[2], cnt = 0; i < modelGeometry[2].boundary[3]; i += cnt)
        {
            cnt = count_consecutive(i, modelGeometry[2].boundary[3], modelGeometry[2].mesh.length);
            print_COPYNODE(f, index.node, index.node + delt, pp[1].node, modelGeometry[2].mesh.length[i], pp[2].node, cnt, 2);
            index.node += cnt * pp[2].node;
        }
        startIndexNode += (modelGeometry[0].boundary[j + 1] - modelGeometry[0].boundary[j]) * pp[0].node;
    }
    index = startIndex;
    index.elm += pp_elm[1] + pp_elm[2];
    for(int i = 2; i < 5; i++)
    {
        print_QUAD(f, index.elm, index.node, pp_node, 1, 2, 1);
        print_COPYELM(f, index.elm, 0, 0, pp_elm[1], pp[1].node, modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] - 1);
        print_COPYELM(f, index.elm, index.elm + (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] - 1) * pp_elm[1], pp_elm[1], pp_elm[2], pp[2].node, modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] - 1);
        index.node += (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i]) * pp[0].node;
        index.elm += pp_elm[0] * (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i] + 1); 
    }
    //y直交面
    fprintf(f, "\n----y\n");
    index = startIndex;
    for(int k = 0; k < 2; k++)
    {
        for(int j = 2; j < 4; j++)
        {
            for(int i = modelGeometry[0].boundary[j], cnt = 0; i < modelGeometry[0].boundary[j + 1] - 1; i += cnt)
            {
                cnt = count_consecutive(i, modelGeometry[0].boundary[j + 1] - 1, modelGeometry[0].mesh.length);
                print_COPYNODE(f, index.node, index.node + (modelGeometry[2].boundary[j + 1] - modelGeometry[2].boundary[j]) * pp[2].node, pp[2].node, modelGeometry[0].mesh.length[i], pp[0].node, cnt, 0);
                index.node += cnt * pp[0].node;
            }
            index.node += pp[0].node;
        }
        index.node = startIndex.node + (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0]) * pp[1].node;
    }
    index = startIndex;
    index.elm += pp_elm[0] + pp_elm[2];
    for(int k = 0; k < 2; k++)
    {
        for(int i = 2; i < 4; i++)
        {
            print_QUAD(f, index.elm, index.node, pp_node, 0, 2, 1);
            print_COPYELM(f, index.elm, 0, 0, pp_elm[0], pp[0].node, modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2] - 1);
            print_COPYELM(f, index.elm, index.elm + (modelGeometry[0].boundary[3] - modelGeometry[0].boundary[2] - 1) * pp_elm[0], pp_elm[0], pp_elm[2], pp[2].node, modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] - 1);
            index.node += (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i]) * pp[0].node;
            index.elm  += pp_elm[0] * (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i] + 1); 
        }
        index.node = startIndex.node + (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0]) * pp_node[1];
        index.elm  = startIndex.elm + pp_elm[0] + pp_elm[2] + (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 1) * pp_elm[1];
    }

    //z直交面
    fprintf(f, "\n----z\n");
    for(int k = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, modelGeometry[1].mesh.length); k < modelGeometry[1].boundary[2]; k++)
    {
        index.node = startIndex.node + k * pp[1].node;
        for(int j = 2; j < 4; j++)
        {
            for(int i = modelGeometry[0].boundary[j], cnt = 0; i < modelGeometry[0].boundary[j + 1] - 1; i += cnt)
            {
                cnt = count_consecutive(i, modelGeometry[0].boundary[j + 1] - 1, modelGeometry[0].mesh.length);
                if(cnt == -1)
                {
                    printf("[ERROR] quad_collumn\n");
                    return ;
                }
                print_COPYNODE(f, index.node, index.node + (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, modelGeometry[0].mesh.length[i], pp[0].node, cnt, 0);
                index.node += cnt * pp[0].node;
            }
            index.node ++;
        }
    }
    fprintf(f, "\n");
    for(int j = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, modelGeometry[0].mesh.length); j <= coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, modelGeometry[0].mesh.length); j++)
    {
        if(j != modelGeometry[0].boundary[3])
        {
            index.node = startIndex.node + (j - modelGeometry[0].boundary[2]) * pp[0].node;
            for(int i = modelGeometry[1].boundary[0], cnt = 0; i < coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, modelGeometry[1].mesh.length) - 1; i += cnt)
            {
                cnt = count_consecutive(i, coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, modelGeometry[1].mesh.length) - 1, modelGeometry[1].mesh.length);
                print_COPYNODE(f, index.node, index.node + (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, modelGeometry[1].mesh.length[i], pp[1].node, cnt, 1);
                index.node += cnt * pp[1].node;
            }
        }
    }
    fprintf(f, "\n");
    for(int j = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, modelGeometry[1].mesh.length); j < modelGeometry[1].boundary[2]; j++)
    {
        index.node = startIndex.node + j * pp[1].node;
        index.elm  = startIndex.elm  + 1 * pp_elm[0] + (j + 1) * pp_elm[1];
        for(int i = 2; i < 4; i++)
        {
            print_QUAD(f, index.elm, index.node, pp_node, 0, 1, 1);
            print_COPYELM(f, index.elm, 0, 0, (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, 1);
            print_COPYELM(f, index.elm, index.elm + (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], pp_elm[0], pp[0].node, modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i] - 1);
            index.node += (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i])     * pp[0].node;
            index.elm  += (modelGeometry[0].boundary[i + 1] - modelGeometry[0].boundary[i] + 1) * pp_elm[0]; 
        }
    }
    fprintf(f, "\n");
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, modelGeometry[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, modelGeometry[0].mesh.length); i++)
    {

        index.node = startIndex.node + (i - modelGeometry[0].boundary[2]) * pp[0].node;
        index.elm  = startIndex.elm  + (i - modelGeometry[0].boundary[2] + 1) * pp_elm[0] + 1 * pp_elm[1];
        if(i >= modelGeometry[0].boundary[3])
        {
            index.elm++;
        }
        print_QUAD(f, index.elm, index.node, pp_node, 0, 1, 1);
        print_COPYELM(f, index.elm, 0, 0, (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2]) * pp[2].node, 1);
        print_COPYELM(f, index.elm, index.elm + (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], (modelGeometry[2].boundary[3] - modelGeometry[2].boundary[2] + 1) * pp_elm[2], pp_elm[1], pp[1].node, coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, modelGeometry[1].mesh.length) - 1);
    }

}

void build_column()
{

}

int main()
{
    /*todo
        ---柱領域---
        内部コンクリート　　要素タイプ
        かぶりコンクリート　要素タイプ

        ---梁領域---
        梁加力治具
        梁

        ---接合要素---
        FILEM

        ---節点結合---
        HOLD

        ---境界条件---
        節点自由度
        節点荷重（柱or梁）

        データチェック
    */

    /*流れ
        inpt.txtに設定を入力
        OUT_FILE_NAMEに書き込み
    */

    /*名称
        input.txt    : 入力ファイル
        out.ffi      : 出力ファイル
        mesh         : 分割情報
        0, 1, 2      : x, y, z
        point        : 格子点
        pp           : ++ （インクリメント）
        times        : 回数
        name         : 節点、要素番号
   */

    /*メモ
        格子の原点は(0, 0, 0)
        主筋入力データの座標は柱左下を原点
        コマンドライン引数でハーフ、フルの選択
        入出力ファイル名の指定
        変数名　キャメルケース
        関数名　スネークケース
    */

    int dir[3] = {'X', 'Y', 'Z'};

    const int startNode = 1;
    const int startElm  = 1;

    struct modelsize rcs;
    struct geometry  modelGeometry[XYZ];
    struct nodeElm pp[XYZ];
    struct nodeElm index;

    /*
    int typq[1];
    int typl[1];
    int typf[1];
    */

    printf("\n[START]\n");

    FILE *fin = fopen(INPUT_FILE_NAME,"r");
    if(fin == NULL)
    {
        printf("[ERROR] input.txt cant open.\n");
        return 1;
    }

    for(int i = 0; i < 3; i++)
    {
        modelGeometry[i].mesh.number = load_mesh(fin, modelGeometry[i].mesh.length, dir[i]);
    }
    rcs = Model_Size_Load(fin);    

    fclose(fin);

    //データチェック
    printf("\n------------------[ DATE CHECK ]------------------\n\n");
    printf("non\n----\n");

    //x, z方向は入力範囲が半分
    //yはハーフモデルのためそのまま
    symmetrical_input(&modelGeometry[0]);
    symmetrical_input(&modelGeometry[2]);

    get_boundary(rcs, modelGeometry);
    
    //試験体情報表示
    console_model_sizes(rcs);

    //メッシュ情報表示
    console_mesh(modelGeometry, dir);
    printf("\n");

    //境界線情報表示
    console_boundary(modelGeometry);

    //ファイルオープン
    FILE *fout = fopen(OUT_FILE_NAME,"w");
    if(fout == NULL)
    {
        printf("[ERROR] out.ffi cant open.\n");
        return 1;
    }

    //雛形
    print_head_template(fout);    

    /*--柱領域---------------------------------------------------------*/
    printf("\n------------------[ COLUMN ]------------------\n");
    //インクリメント
    pp[0].node = 1;
    pp[1].node = modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2] + 2;
    pp[2].node = pp[1].node * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 1);
    pp[0].elm  = 1;
    pp[1].elm  = modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2];
    pp[2].elm  = pp[1].elm * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0]);
    console_increment(pp);

    //六面体要素
    index.node = startNode;
    index.elm  = startElm;
    console_index(index, "concrete start");
    index = add_concrete(fout, index, pp, modelGeometry);
    //かぶり、内部コンクリート要素番号

    //線材要素
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    console_index(index, "rebar start");
    index = add_reber(fout, pp, rcs, modelGeometry, index);
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);

    //四辺形要素
    console_index(index, "quad column start");
    quad_collumn(fout, index, modelGeometry, pp, rcs);
    //六面体要素

    //四辺形要素

    //フィルム要素

    print_tail_template(fout);
    fclose(fout);

    printf("\n[END]\n");

    return 0;
}
