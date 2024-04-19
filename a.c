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
    double cordi[2];
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

struct increment
{
    int node[XYZ];
    int elm [XYZ];
};

struct startEnd
{
    int start[XYZ];
    int end  [XYZ];
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
        int result = fscanf(f, " X=%lf Y=%lf", &temp.rebar[index - 1].cordi[0], &temp.rebar[index - 1].cordi[1]);
        if (result == EOF) {
            // ファイルの終わりに達した場合はループを終了
            break;
        } else if (result == 0) {
            // 数値の読み取りに失敗した場合はエラーを表示して終了
            printf("[ERROR] load mesh\n");
            temp.rebar[index].cordi[0] = -999;
            temp.rebar[index].cordi[1] = -999;
            break;
        }
        temp.rebar[index - 1].cordi[0] += (temp.beam.span - temp.column.depth) / 2;
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
        printf("%7.1f", rcs.rebar[i].cordi[0]);
    }
    printf("\nY    ");
    for(int i = 0; i < REBAR_MAX; ++i)
    {
        printf("%7.1f", rcs.rebar[i].cordi[1]);
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
    "TYPF :(  1)  MATJ(  2)  AXIS(   )\n"
    "\n----+----\n"
    "MATS :(  1)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATS :(  2)  ES=2.05   (E+5) PR=0.3   SY=295    HR=0.01  ALP=      (E-5)\n"
    "MATC :(  1)  EC=2.05   (E+4) PR=0.2   FC=30     FT=3     ALP=      (E-5)\n"
    "MATJ :(  1)  TYPE=(4) (1:CRACK  2:BOND  3:GENERIC  4:RIGID  5:DASHPOT)\n"
    "MATJ :(  2)  TYPE=(4) (1:CRACK  2:BOND  3:GENERIC  4:RIGID  5:DASHPOT)\n"
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

void print_BEAM(FILE *f, int elmIndex, int nodeIndex, int nodePp, int typb)
{
    fprintf(f, "BEAM :(%5d)(%5d:%5d) TYPB(%3d)  Y-NODE(     )\n", elmIndex, nodeIndex, nodeIndex + nodePp, typb);
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
    fprintf(f, "FILM :(%5d)(%5d:%5d:%5d:%5d:%5d:%5d:%5d:%5d) TYPF(%3d)\n", elmIndex, face1, face1 + nodePp[dir1], face1 + nodePp[dir1] + nodePp[dir2], face1 + nodePp[dir2], face2, face2 + nodePp[dir1], face2 + nodePp[dir1] + nodePp[dir2], face2 + nodePp[dir2], typf);
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

int plot_node(FILE *f, int nodeIndexStart, struct startEnd point, struct geometry geo[], int nodePp[])
{
    int dir = 0;
    int _dir;
    int index = nodeIndexStart;
    int delt = 0;
    double cordi[XYZ];
    //節点定義
    for(int i = 0; i < XYZ; i++)
    {
        cordi[i] = point_to_coordinate(point.start[i], geo[i].mesh.length);
    }
    print_NODE(f, index, cordi);
    //節点コピー
    for(dir = 0; dir < XYZ; dir++)
    {
        if(point.end[dir] > point.start[dir])
        {
            index = nodeIndexStart;
            for(int i = point.start[dir], cnt = 0; i < point.end[dir]; i += cnt)
            {
                cnt = count_consecutive(i, point.end[dir], geo[dir].mesh.length);
                if(cnt < 0)
                {
                    printf("[ERROR] X\n");
                    return -1;
                }
                print_COPYNODE(f, index, 0, 0, geo[dir].mesh.length[i], nodePp[dir], cnt, dir);
                index += cnt * nodePp[dir];
            }
            delt = index - nodeIndexStart;
            _dir = dir;
            dir++;
            break;
        }
    }
    for(; dir < XYZ; dir++)
    {
        if(point.end[dir] > point.start[dir])
        {
            index = nodeIndexStart;
            for(int i = point.start[dir], cnt = 0; i < point.end[dir]; i += cnt)
            {
                cnt = count_consecutive(i, point.end[dir], geo[dir].mesh.length);
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
    if(dir < XYZ && point.end[dir] > point.start[dir])
    {
        for(int j = point.start[1]; j <= point.end[1]; j++)
        {
            index = nodeIndexStart + (j - point.start[1]) * nodePp[1];
            for(int i = point.start[2], cnt = 0; i < point.end[2]; i += cnt)
            {
                cnt = count_consecutive(i, point.end[2], geo[2].mesh.length);
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
    return nodeIndexStart + (point.end[0] - point.start[0]) * nodePp[0] + (point.end[1] - point.start[1]) * nodePp[1] + (point.end[2] - point.start[2]) * nodePp[2];
}


/*--要素生成-----------------------------------------------*/
void generate_beam(FILE *f, struct increment pp, struct startEnd point, struct nodeElm startIndex, struct geometry geo[], int typb)
{
    int dir;
    for(int i = 0; i < XYZ; i++)
    {
        if(point.end[i] != point.start[i])
        {
            dir = i;
            break;
        }
    }
    //節点定義
    plot_node(f, startIndex.node, point, geo, pp.node);
    //要素定義
    print_BEAM(f, startIndex.elm, startIndex.node, pp.node[dir], typb);
    print_COPYELM(f, startIndex.elm, 0, 0, 1, pp.node[dir], point.end[dir] - point.start[dir] - 1);
}

void generate_quad(FILE *f, struct increment pp, struct startEnd point, struct nodeElm startIndex, struct geometry geo[], int typq)
{
    int dir1 = 0, dir2 = 1;
    for(int i = 0; i < 2; i++)
    {
        if(point.end[i] == point.start[i])
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
    plot_node(f, startIndex.node, point, geo, pp.node);
    //要素定義
    print_QUAD(f, startIndex.elm, startIndex.node, pp.node, dir1, dir2, typq);
    print_COPYELM(f, startIndex.elm, 0, 0, pp.elm[dir1], pp.node[dir1], point.end[dir1] - point.start[dir1] - 1);
    print_COPYELM(f, startIndex.elm, startIndex.elm + (point.end[dir1] - point.start[dir1] - 1) * pp.elm[dir1], pp.elm[dir1], pp.elm[dir2], pp.node[dir2], point.end[dir2] - point.start[dir2] - 1);
}

struct nodeElm generate_hexa(FILE *f, struct nodeElm startIndex, struct increment pp, struct startEnd point, struct geometry geo[], int TYPH)
{
    int delt;
    struct nodeElm index;
    struct nodeElm returnIndex;
    index.node = startIndex.node;
    index.elm  = startIndex.elm;
    //節点定義
    returnIndex.node = plot_node(f, startIndex.node, point, geo, pp.node);
    fprintf(f, "\n");
    //要素定義
    print_HEXA(f, startIndex.elm, startIndex.node, pp.node, TYPH);
    //要素コピー
    //x
    returnIndex.elm = print_COPYELM(f, index.elm, 0, 0, pp.elm[0], pp.node[0], point.end[0] - point.start[0] - 1);
    //y
    delt = (point.end[0] - point.start[0] - 1) * pp.elm[0];
    returnIndex.elm = print_COPYELM(f, index.elm, index.elm + delt, pp.elm[0], pp.elm[1], pp.node[1], point.end[1] - point.start[1] - 1);
    //z
    for(int i = 0; i < point.end[1] - point.start[1]; i++)
    {
        returnIndex.elm = print_COPYELM(f, index.elm + pp.elm[1] * i, index.elm + delt + pp.elm[1] * i, pp.elm[0], pp.elm[2], pp.node[2], point.end[2] - point.start[2] - 1);
    }
    return returnIndex;
}

void generate_line(FILE *f, int elmIndex, int hexaNodeIndex, int beamNodeIndex, int nodePp, int set)
{
    const int elmPPz = 1;
    int index = elmIndex;
    print_LINE(f, index, hexaNodeIndex, beamNodeIndex, nodePp);
    print_COPYELM(f, elmIndex, 0, 0, elmPPz, nodePp, set);
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
            generate_beam(f, pp, point, index, geo, typb);
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
    int typh[5] = {1, 2, 3, 2, 1};
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
            returnIndex = generate_hexa(f, index, pp, point, geo, typh[j]);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1) * pp.node[0];
            index.elm  += (geo[0].boundary[i + 1] - geo[0].boundary[i])     * pp.elm [0];
        }
        index.node = startIndex.node + (geo[2].boundary[j + 1] + j + 1) * pp.node[2];
        index.elm  = startIndex.elm  +  geo[2].boundary[j + 1]          * pp.elm [2];
    }
    return returnIndex;
}

struct nodeElm quad_joint(FILE *f, struct nodeElm startIndex, struct geometry geo[], struct increment pp, struct modelsize rcs, int startNode)
{
    int cnt = 0;
    double cordi[XYZ];
    int filmIndex;
    struct nodeElm index;
    int ppElm[3];
    ppElm[0] = 1;
    ppElm[1] = pp.elm[1] + 4;
    ppElm[2] = ppElm[1] * (geo[1].boundary[2] - geo[1].boundary[0] + 2);

    int filmStart = startIndex.elm + (geo[0].boundary[4] - geo[0].boundary[2] + 3) * (geo[1].boundary[2] - geo[1].boundary[0] + 2) * (geo[2].boundary[3] - geo[2].boundary[2] + 2);
    filmStart = next_index(filmStart);

    printf("filmstart -> %d\n", filmStart);
    index = startIndex;
    int startIndexNode = startIndex.node;
    const int concreteSteelDelt = startIndex.node - (startNode + (geo[2].boundary[2] + 2) * pp.node[2]);
    fprintf(f, "\n----COLUMN QUAD----\n");
    //x直交面
    fprintf(f, "\n----x\n");
    for(int j = 2; j < 5; j++)
    {
        index.node = startIndexNode;
        cordi[0] = point_to_coordinate(geo[0].boundary[j], geo[0].mesh.length);
        cordi[1] = point_to_coordinate(geo[1].boundary[0], geo[1].mesh.length);
        cordi[2] = point_to_coordinate(geo[2].boundary[2], geo[2].mesh.length);
        print_NODE(f, index.node, cordi);
        for(int i = geo[1].boundary[0]; i < geo[1].boundary[2]; i += cnt)
        {
            cnt = count_consecutive(i, geo[1].boundary[2], geo[1].mesh.length);
            print_COPYNODE(f, index.node, 0, 0, geo[1].mesh.length[i], pp.node[1], cnt, 1);
            index.node += cnt * pp.node[1];
        }
        int delt = index.node - startIndexNode;
        index.node = startIndexNode;
        for(int i = geo[2].boundary[2], cnt = 0; i < geo[2].boundary[3]; i += cnt)
        {
            cnt = count_consecutive(i, geo[2].boundary[3], geo[2].mesh.length);
            print_COPYNODE(f, index.node, index.node + delt, pp.node[1], geo[2].mesh.length[i], pp.node[2], cnt, 2);
            index.node += cnt * pp.node[2];
        }
        startIndexNode += (geo[0].boundary[j + 1] - geo[0].boundary[j]) * pp.node[0];
    }
    //四辺形要素
    index = startIndex;
    index.elm += ppElm[1] + ppElm[2];
    for(int i = 2; i < 5; i++)
    {
        print_QUAD(f, index.elm, index.node, pp.node, 1, 2, 1);
        print_COPYELM(f, index.elm, 0, 0, ppElm[1], pp.node[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        print_COPYELM(f, index.elm, index.elm + (geo[1].boundary[2] - geo[1].boundary[0] - 1) * ppElm[1], ppElm[1], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i]) * pp.node[0];
        index.elm += ppElm[0] * (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1); 
    }
    //film要素
    fprintf(f, "\n----\n");
    filmIndex = filmStart + ppElm[1] + 2 * ppElm[2];
    index = startIndex;
    int delt = concreteSteelDelt;
    for(int i = 0; i < 2; i++)
    {
        print_FILM(f, filmIndex, index.node - delt, index.node, pp.node, 2, 1, 1);
        print_COPYELM(f, filmIndex, 0, 0, ppElm[1], pp.node[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        print_COPYELM(f, filmIndex, filmIndex + (geo[1].boundary[2] - geo[1].boundary[0] - 1) * ppElm[1], ppElm[1], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        filmIndex  += geo[0].boundary[3] - geo[0].boundary[2] + 1;
        index.node += geo[0].boundary[3] - geo[0].boundary[2]; 
        print_FILM(f, filmIndex, index.node - delt, index.node, pp.node, 1, 2, 1);
        print_COPYELM(f, filmIndex, 0, 0, ppElm[1], pp.node[1], geo[1].boundary[2] - geo[1].boundary[0] - 1);
        print_COPYELM(f, filmIndex, filmIndex + (geo[1].boundary[2] - geo[1].boundary[0] - 1) * ppElm[1], ppElm[1], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        filmIndex += ppElm[0];
        delt -= pp.node[0];
    }

    //y直交面
    fprintf(f, "\n----y\n");
    index = startIndex;
    for(int k = 0; k < 2; k++)
    {
        for(int j = 2; j < 4; j++)
        {
            for(int i = geo[0].boundary[j], cnt = 0; i < geo[0].boundary[j + 1] - 1; i += cnt)
            {
                cnt = count_consecutive(i, geo[0].boundary[j + 1] - 1, geo[0].mesh.length);
                print_COPYNODE(f, index.node, index.node + (geo[2].boundary[j + 1] - geo[2].boundary[j]) * pp.node[2], pp.node[2], geo[0].mesh.length[i], pp.node[0], cnt, 0);
                index.node += cnt * pp.node[0];
            }
            index.node += pp.node[0];
        }
        index.node = startIndex.node + (geo[1].boundary[2] - geo[1].boundary[0]) * pp.node[1];
    }
    //四辺形要素
    index = startIndex;
    index.elm += ppElm[0] + ppElm[2];
    for(int k = 0; k < 2; k++)
    {
        for(int i = 2; i < 4; i++)
        {
            print_QUAD(f, index.elm, index.node, pp.node, 0, 2, 1);
            print_COPYELM(f, index.elm, 0, 0, ppElm[0], pp.node[0], geo[0].boundary[3] - geo[0].boundary[2] - 1);
            print_COPYELM(f, index.elm, index.elm + (geo[0].boundary[3] - geo[0].boundary[2] - 1) * ppElm[0], ppElm[0], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i]) * pp.node[0];
            index.elm  += ppElm[0] * (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1); 
        }
        index.node = startIndex.node + (geo[1].boundary[2] - geo[1].boundary[0]) * pp.node[1];
        index.elm  = startIndex.elm + ppElm[0] + ppElm[2] + (geo[1].boundary[2] - geo[1].boundary[0] + 1) * ppElm[1];
    }
    //film要素
    fprintf(f, "\n----\n");
    filmIndex = filmStart + ppElm[0] + 2 * ppElm[2];
    index = startIndex;
    delt = concreteSteelDelt;
    for(int i = 0; i < 2; i++)
    {
        print_FILM(f, filmIndex, index.node - delt, index.node, pp.node, 0, 2, 1);
        print_COPYELM(f, filmIndex, 0, 0, ppElm[0], pp.node[0], geo[0].boundary[3] - geo[0].boundary[2] - 1);
        print_COPYELM(f, filmIndex, filmIndex + (geo[0].boundary[3] - geo[0].boundary[2] - 1) * ppElm[0], ppElm[0], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        filmIndex  += (geo[1].boundary[2] - geo[1].boundary[0] + 1) * ppElm[1];
        index.node += (geo[1].boundary[2] - geo[1].boundary[0]) * pp.node[1]; 
        print_FILM(f, filmIndex, index.node - delt, index.node, pp.node, 2, 0, 1);
        print_COPYELM(f, filmIndex, 0, 0, ppElm[0], pp.node[0], geo[0].boundary[3] - geo[0].boundary[2] - 1);
        print_COPYELM(f, filmIndex, filmIndex + (geo[0].boundary[3] - geo[0].boundary[2] - 1) * ppElm[0], ppElm[0], ppElm[2], pp.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
        filmIndex  = filmStart + ppElm[0] + ppElm[2] + (geo[0].boundary[3] - geo[0].boundary[2] + 1) * ppElm[0];
        index.node = startIndex.node + (geo[0].boundary[3] - geo[0].boundary[2]) * ppElm[0];
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
    for(int j = coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length); j < geo[1].boundary[2]; j++)
    {
        index.node = startIndex.node + j * pp.node[1];
        index.elm  = startIndex.elm  + 1 * ppElm[0] + (j + 1) * ppElm[1];
        for(int i = 2; i < 4; i++)
        {
            print_QUAD(f, index.elm, index.node, pp.node, 0, 1, 1);
            print_COPYELM(f, index.elm, 0, 0, (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppElm[2], (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], 1);
            print_COPYELM(f, index.elm, index.elm + (geo[2].boundary[3] -geo[2].boundary[2] + 1) * ppElm[2], (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppElm[2], ppElm[0], pp.node[0], geo[0].boundary[i + 1] - geo[0].boundary[i] - 1);
            index.node += (geo[0].boundary[i + 1] - geo[0].boundary[i])     * pp.node[0];
            index.elm  += (geo[0].boundary[i + 1] - geo[0].boundary[i] + 1) * ppElm[0]; 
        }
    }
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); i++)
    {

        index.node = startIndex.node + (i - geo[0].boundary[2]) * pp.node[0];
        index.elm  = startIndex.elm  + (i - geo[0].boundary[2] + 1) * ppElm[0] + 1 * ppElm[1];
        if(i >= geo[0].boundary[3])
        {
            index.elm++;
        }
        print_QUAD(f, index.elm, index.node, pp.node, 0, 1, 1);
        print_COPYELM(f, index.elm, 0, 0, (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppElm[2], (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], 1);
        print_COPYELM(f, index.elm, index.elm + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppElm[2], (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppElm[2], ppElm[1], pp.node[1], coordinate_to_point((rcs.column.width - rcs.beam.width) / 2, geo[1].mesh.length) - 1);
    }
    //film要素
    fprintf(f, "\n----\n");
    index = startIndex;
    delt = concreteSteelDelt;
    for(int j = 2; j < 4; j++)
    {
        for(int i = geo[1].boundary[1]; i < geo[1].boundary[2]; i++)
        {
            index.node = (geo[0].boundary[j] - geo[0].boundary[2]) * pp.node[0] + startIndex.node + i * pp.node[1];
            filmIndex = filmStart + ppElm[0] + (i + 1) * ppElm[1] + (geo[0].boundary[j] - geo[0].boundary[2]) * ppElm[0];
            if(j >= 3)
            {
                filmIndex  += ppElm[0];
                delt -= pp.node[0];
            }
            print_FILM(f, filmIndex, index.node - delt - pp.node[2], index.node, pp.node, 1, 0, 1);
            print_FILM(f, filmIndex + ppElm[2], index.node - delt, index.node, pp.node, 0, 1, 1);
            print_COPYELM(f, filmIndex, filmIndex + ppElm[2], ppElm[2], ppElm[0], pp.node[0], geo[0].boundary[j + 1] - geo[0].boundary[j] - 1);
            
            print_FILM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * ppElm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 1);
            print_FILM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * ppElm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 1, 0, 1);
            print_COPYELM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * ppElm[2], filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * ppElm[2], ppElm[2], ppElm[0], pp.node[0], geo[0].boundary[j + 1] - geo[0].boundary[j] - 1);
        }
    }
    index = startIndex;
    delt = concreteSteelDelt;
    for(int i = coordinate_to_point((rcs.beam.span - rcs.xbeam.width) / 2, geo[0].mesh.length); i < coordinate_to_point((rcs.beam.span + rcs.xbeam.width) / 2, geo[0].mesh.length); i++)
    {
        index.node = startIndex.node + (i - geo[0].boundary[2]) * pp.node[0];
        filmIndex = filmStart + (i - geo[0].boundary[2] + 1) * ppElm[0] + ppElm[1];
        if(i >= geo[0].boundary[3])
        {
            delt -= pp.node[0];
        }
        print_FILM(f, filmIndex, index.node - delt - pp.node[2], index.node, pp.node, 0, 1, 1);
        print_FILM(f, filmIndex + ppElm[2], index.node - delt, index.node, pp.node, 1, 0, 1);
        print_COPYELM(f, filmIndex, filmIndex + ppElm[2], ppElm[2], ppElm[1], pp.node[1], geo[1].boundary[1] - 1);

        print_FILM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * ppElm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 0, 1, 1);
        print_FILM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * ppElm[2], index.node - delt + (geo[2].boundary[3] - geo[2].boundary[2] + 1) * pp.node[2], index.node + (geo[2].boundary[3] - geo[2].boundary[2]) * pp.node[2], pp.node, 1, 0, 1);
        print_COPYELM(f, filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 2) * ppElm[2], filmIndex + (geo[2].boundary[3] - geo[2].boundary[2] + 3) * ppElm[2], ppElm[2], ppElm[1], pp.node[1], geo[1].boundary[1] - 1);
    }
    index.node  = startIndex.node;
    index.node += (geo[0].boundary[4] - geo[0].boundary[2] + 1) * (geo[1].boundary[2] - geo[1].boundary[0] + 1) * (geo[2].boundary[3] - geo[2].boundary[2] + 1);
    index.elm   = filmStart;
    index.elm  += (geo[0].boundary[4] - geo[0].boundary[2] + 4) * (geo[1].boundary[2] - geo[1].boundary[0] + 2) * (geo[2].boundary[3] - geo[2].boundary[2] + 4);
    return index;
}

void hexa_beam(FILE *f, struct nodeElm startIndex, struct increment pp, struct geometry geo[])
{
    const int typh = 1;
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
        generate_hexa(f, index, pp, point, geo, typh);
    }
}

void quad_beam(FILE *f, struct nodeElm startIndex, struct increment ppColumn, struct increment ppBeam, struct geometry geo[], int jointStartNode)
{
    const int typq = 1;
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
            plot_node(f, index.node, point, geo, ppBeam.node);
        }
        point.start[1] = geo[1].boundary[2];
        point.start[2] = geo[2].boundary[2] + 1;
        point.end  [2] = geo[2].boundary[3] - 1;
        index.node = startIndex.node + (geo[0].boundary[j] + 1) * ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1] + ppBeam.node[2];
        plot_node(f, index.node, point, geo, ppBeam.node);
    }
    //要素定義
    //接合部分
    quadDelt    = (geo[0].boundary[4] - geo[0].boundary[2] + 2) * ppBeam.node[0];
    jointDelt   = (geo[0].boundary[4] - geo[0].boundary[2])     * ppColumn.node[0];
    elmDelt     = (geo[0].boundary[4] - geo[0].boundary[2] + 1) * ppBeam.elm[0];

    index.node  = startIndex.node + (geo[0].boundary[2] - 1) * ppBeam.node[0];
    index.elm   = startIndex.elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0];
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm, index.node, jointNode + geo[1].boundary[1] * ppColumn.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1], index.node + ppBeam.node[1], typq); 
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt, jointNode + geo[1].boundary[1] * ppColumn.node[1] + jointDelt, index.node + quadDelt, index.node + ppBeam.node[1] + quadDelt, jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + jointDelt, typq); 
    index.node += (geo[2].boundary[3] - geo[2].boundary[2]) * ppBeam.node[2];
    index.elm  += (geo[2].boundary[3] - geo[2].boundary[2] + 1) * ppBeam.elm[2];
    jointNode  += (geo[2].boundary[3] - geo[2].boundary[2]) * ppColumn.node[2];
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm, index.node, jointNode + geo[1].boundary[1] * ppColumn.node[1], jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1], index.node + ppBeam.node[1], typq); 
    fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt, jointNode + geo[1].boundary[1] * ppColumn.node[1] + jointDelt, index.node + quadDelt, index.node + ppBeam.node[1] + quadDelt, jointNode + (geo[1].boundary[1] + 1) * ppColumn.node[1] + jointDelt, typq); 
    
    jointNode  = jointStartNode;
    index.node = startIndex.node + (geo[0].boundary[2] - 1) * ppBeam.node[0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = startIndex.elm  + (geo[0].boundary[2] - 1) * ppBeam.elm [0]+ (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm [1] + ppBeam.elm [2];
    for(int i = 0; i < geo[2].boundary[3] - geo[2].boundary[2]; i++)
    {
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm, index.node, jointNode + geo[1].boundary[2] * ppColumn.node[1], jointNode + geo[1].boundary[2] * ppColumn.node[1] + ppColumn.node[2], index.node + ppBeam.node[2], typq); 
        fprintf(f, "QUAD :(%5d)(%5d:%5d:%5d:%5d) TYPQ(%3d)\n", index.elm + elmDelt, jointNode + geo[1].boundary[2] * ppColumn.node[1] + jointDelt, index.node + quadDelt, index.node + ppBeam.node[2] + quadDelt, jointNode + geo[1].boundary[2] * ppColumn.node[1] + ppColumn.node[2] + jointDelt, typq); 
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
        print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 1, typq);
        print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 1, typq);    
        print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
    }
    index.node = startIndex.node + ppBeam.node[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.node[1];
    index.elm  = startIndex.elm  + ppBeam.elm [0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    print_QUAD(f, index.elm, index.node, ppBeam.node, 0, 2, typq);
    print_QUAD(f, index.elm + elmDelt, index.node + quadDelt, ppBeam.node, 0, 2, typq);    
    print_COPYELM(f, index.elm, index.elm + elmDelt, elmDelt, ppBeam.elm[0], ppBeam.node[0], geo[0].boundary[2] - geo[0].boundary[1] - 2);
    
    index.elm = startIndex.elm + ppBeam.elm[0] + (geo[1].boundary[2] - geo[1].boundary[1]) * ppBeam.elm[1] + ppBeam.elm[2];
    elmDelt = (geo[0].boundary[2] - geo[0].boundary[1] - 2) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
    index.elm += (geo[0].boundary[4] - geo[0].boundary[1] + 1) * ppBeam.elm[0];
    print_COPYELM(f, index.elm, index.elm + elmDelt, ppBeam.elm[0], ppBeam.elm[2], ppBeam.node[2], geo[2].boundary[3] - geo[2].boundary[2] - 1);
}

void build_column()
{

}

int main()
{
    /*todo

        節点要素番号を返す
        番号の表示

        ---要素タイプ---
        内部コンクリート
        かぶりコンクリート
        鉄骨梁
        film要素

        ---節点結合---
        HOLD

        ---境界条件---
        節点自由度
        節点強制変位（柱or梁）

        データチェック
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
    int jointStartNode;
    const int startNode = 1;
    const int startElm  = 1;
    struct modelsize rcs;
    struct geometry  modelGeometry[XYZ];
    struct increment ppColumn;
    struct increment ppBeam;
    struct nodeElm   index;

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

    //x, y, z方向は入力範囲が半分
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

    //インクリメント
    ppColumn.node[0] = 1;
    ppColumn.node[1] = modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2] + 2;
    ppColumn.node[2] = ppColumn.node[1] * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0] + 1);
    ppColumn.elm [0] = 1;
    ppColumn.elm [1] = modelGeometry[0].boundary[4] - modelGeometry[0].boundary[2];
    ppColumn.elm [2] = ppColumn.elm[1] * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[0]);
    console_increment(ppColumn);
    ppBeam.node[0] = 1;
    ppBeam.node[1] = modelGeometry[0].boundary[6] + 1;
    ppBeam.node[2] = ppBeam.node[1] * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[1] + 1);
    ppBeam.elm [0] = 1;
    ppBeam.elm [1] = modelGeometry[0].boundary[6];
    ppBeam.elm [2] = ppBeam.elm[1] * (modelGeometry[1].boundary[2] - modelGeometry[1].boundary[1] + 1);
    console_increment(ppBeam);

    //ファイルオープン
    FILE *fout = fopen(OUT_FILE_NAME,"w");
    if(fout == NULL)
    {
        printf("[ERROR] out.ffi cant open.\n");
        return 1;
    }

    //雛形
    print_head_template(fout);    
    printf("\n------------------[ COLUMN ]------------------\n");

    //六面体要素
    index.node = startNode;
    index.elm  = startElm;
    console_index(index, "concrete start");
    index = add_concrete(fout, index, ppColumn, modelGeometry);

    //線材要素
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    console_index(index, "rebar start");
    index = add_reber(fout, ppColumn, rcs, modelGeometry, index);
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);
    jointStartNode = index.node;
    printf("joint -> %d\n", jointStartNode);

    printf("\n------------------[ BEAM ]------------------\n");
    //四辺形要素
    console_index(index, "quad column start");
    index = quad_joint(fout, index, modelGeometry, ppColumn, rcs, startNode);
    index.node  = next_index(index.node);
    index.elm   = next_index(index.elm);

    //六面体要素
    console_index(index, "hexa beam");
    hexa_beam(fout, index, ppBeam, modelGeometry);

    //四辺形要素
    quad_beam(fout, index, ppColumn, ppBeam, modelGeometry, jointStartNode);
    
    //かぶり、内部コンクリート要素番号

    print_tail_template(fout);
    fclose(fout);

    printf("\n[END]\n");

    return 0;
}
