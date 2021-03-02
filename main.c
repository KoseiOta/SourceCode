////////////////
// メイン関数 //
////////////////

//////////////////////////
// インクルードファイル //
//////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#define N_MAX 7000 //ノードの最大数
#define TRUE 1     //trueかfalseかのマクロ定義
#define FALSE 0
#define SIM_NUM 5 //シミュレーション回数
#define RA_STEP_NUM 18        // 進入制限エリアの数
#define DO_ROBOT_EMPTY_TURN 1 //空回し時にロボットを動かすかどうか
#define USE_FS 1  //FSを用いるかどうか ロボット使う時は0に

////////////////////
// グローバル変数 //
////////////////////

double r = 100.0; //送信範囲 [m]
double v = 10.0;  //端末の移動速度の絶対値 [m/s]
double dt = 0.1;  //移動時間ステップ [s]

double D = 125.0; //交差点と送信可能エリアの距離 [m]

double L = 250.0;   //送信可能エリアTAの長さ[m]
int block_num = 12; //区画の数
double A_i;         //交差点の区間の長さ

double Ax; //サービスエリアの横幅 [m]
double Ay; //サービスエリアの縦幅 [m]

double lambda_W; //右から進入するノードの密度 [m^-1]
double lambda_N; //下から進入するノードの密度 [m^-1]
double lambda_E; //左から進入するノードの密度 [m^-1]
double lambda_S; //上から進入するノードの密度 [m^-1]

int d_ta = 3;                //FSからTAまでの交差点数
int d_ra[RA_STEP_NUM] = {3}; //FSから進入制限エリアの端までの交差点数
int ta_num;                  //TAの数（FS，RAを囲うように複数配置）
int ra_num[RA_STEP_NUM];     //進入制限エリアを表す構造体の数
int ra_step_num = 1;
int count_area_num;

int Info_StartQ; // 情報が入ってくるかどうか

double q = 0.9;                         //RA1を提示する確率（RAを併用するときだけ使う）
double p_avo[RA_STEP_NUM] = {1.0, 1.0}; //情報を受け取ったノードが事故現場を避ける確率

int ID; //次に発生するノードのノードID

double Twait_max; //シミュレーション時間[s]
double Twait = 0.0;        //待ち時間[s]

int Sim_Flag = 0; //空回しが終了したかどうか
//情報を持ってRAに進入したノードの数
int num_nodes_info[RA_STEP_NUM];
//情報を持たずにRAに進入したノードの数
int num_nodes_no_info[RA_STEP_NUM];

//不必要な情報を受け取ったノードの数
int num_recieved_unnecessary_info[RA_STEP_NUM];
//不必要でない情報受け取ったノードの数
int num_recieved_necessary_info[RA_STEP_NUM];
//RAに進入する可能性のあったノードの数
int num_not_having_passable_route[1000] = {0};

int count_num_nodes_no_info[1000] = {0}; //情報を持たずにRAに進入したノードの数を設定時間ごとに数える用の配列

int Cross[100] = {0}; //各交差点からRAに進入するノード数を計測
int TotalCross[1000] = {0}; //各交差点から進入したノード数をSIM_NUM回分カウント
 
int **num_nodes_info_road;            //各道路区間に情報を持って存在するノードの数
int **num_nodes_no_info_road;         //各道路区間に情報を持たずに存在するノードの数
int **num_nodes_info_road2[4];        //各道路区間に情報を持って存在するノードの数（進行方向ごと）
int **num_nodes_no_info_road2[4];     //各道路区間に情報を持たずに存在するノードの数（進行方向ごと）
int **num_nodes_info_intersection;    //各交差点周辺に情報を持って存在するノードの数
int **num_nodes_no_info_intersection; //各交差点周辺に情報を持たず存在するノードの数

double epsilon = 0.0001; //double型の等値比較の際に使用

//ロボット関連
#define R_MAX 16                                           //ロボット数の上限
#define R_SECTION_MAX 50                                   //ロボットの経路区間の上限数
int num_robot = 0;                                         //使用するロボットの数。デフォルトは0。
int robot_condition_num;                                   //読み込むテキストの条件番号。1~5まで指定可能。
int print_Twait_max;                                       //結果ファイル名用の変数。
double **buf_robot_data;                                   //ロボットのデータの一時格納
int num_recieved_unnecessary_info_from_robot[RA_STEP_NUM]; //不必要な情報をロボットから受け取った数
int num_recieved_necessary_info_from_robot[RA_STEP_NUM];   //不必要でない情報をロボットから受け取った数

//結果を出力するファイルのファイルポインタ
FILE *fp, *fp_num_nodes_road;

int sim_time;
///////////////////////////////
// ヘッダファイルの読み込み  //
///////////////////////////////
#include "Common.h"
#include "Move.h"
#include "Transmit.h"
#include "Node_Dist.h"

////////////////
// メイン関数 //
////////////////

int main(int argc, char *argv[])
{
  int i_node, i, j, k;
  int node_num;
  int PlotNodes_Flag = 0; //PlotNodesのONOFFをコマンドラインで操作するためのフラグ
  double lambda = 0.0;

  unsigned int Seed;                        //シード値
  char fname[30], fname_num_nodes_road[40]; //出力用のファイル名を格納

  //コマンドライン引数の取得
  //1つ目はノードの密度
  if (argc > 1)
    lambda = atof(argv[1]);

  //2つめはロボットの条件番号
  if (argc > 2)
    robot_condition_num = atoi(argv[2]);

  //3つめはロボットの数
  if (argc > 3)
    num_robot = atoi(argv[3]);

  //4つ目はシミュレーション時間
  if (argc > 4)
    Twait_max = atof(argv[4]);

  //5つ目はPlotNodes()のONOFF(0:OFF 1:ON)
  if (argc > 5)
    PlotNodes_Flag = atof(argv[5]); //何も入力しない場合はOFF


  //カウンタ用の擬似的なRAを設定
  count_area_num = d_ra[ra_step_num - 1]; 
  for (i = 1; i <= count_area_num; i++)
  {
    d_ra[ra_step_num + i - 1] = i; 
  }
  
  A_i = 2 * D + L;      //交差点間の距離 [m]
  Ax = block_num * A_i; //サービスエリアの横幅 [m]
  Ay = Ax;              //サービスエリアの縦幅 [m]

  ta_num = 29; //TAの数 TA, RAを変更する場合はここも変更

  //結果出力ファイルのファイル名生成とファイルオープン
  print_Twait_max = Twait_max;
  sprintf(fname, "c%d_r%d_%f_%d_", robot_condition_num, num_robot, lambda, print_Twait_max);
  for (i = 0; i < ra_step_num; i++)
  {
    sprintf(fname, "%sy%d", fname, d_ra[i]);
  }
  sprintf(fname_num_nodes_road, "%s_num_node_road.txt", fname);
  sprintf(fname, "%s.txt", fname);

  if ((fp = fopen((const char *)fname, "w")) == NULL)
  {
    printf("file open error\n");
    exit(EXIT_FAILURE);
  }
  if ((fp_num_nodes_road = fopen((const char *)fname_num_nodes_road, "w")) == NULL)
  {
    printf("file open error\n");
    exit(EXIT_FAILURE);
  }

  //時間の取得と出力
  time_t timer;
  struct tm *local;

  timer = time(NULL);
  local = localtime(&timer);
  printf("%4d/", local->tm_year + 1900);
  printf("%2d/", local->tm_mon + 1);
  printf("%2d ", local->tm_mday);
  printf("%2d:", local->tm_hour);
  printf("%2d\n", local->tm_min);
  fprintf(fp, "%4d/", local->tm_year + 1900);
  fprintf(fp, "%2d/", local->tm_mon + 1);
  fprintf(fp, "%2d ", local->tm_mday);
  fprintf(fp, "%2d:", local->tm_hour);
  fprintf(fp, "%2d\n", local->tm_min);
  fprintf(fp_num_nodes_road, "%4d/", local->tm_year + 1900);
  fprintf(fp_num_nodes_road, "%2d/", local->tm_mon + 1);
  fprintf(fp_num_nodes_road, "%2d ", local->tm_mday);
  fprintf(fp_num_nodes_road, "%2d:", local->tm_hour);
  fprintf(fp_num_nodes_road, "%2d\n", local->tm_min);
  //実行端末のホスト名の取得と出力
  char hostname[256];
  //gethostname(hostname, sizeof(hostname));
  fprintf(fp, "hostname : %s\n", hostname);
  fprintf(fp_num_nodes_road, "hostname : %s\n", hostname);
  printf("------------------------------------------\n");
  fprintf(fp, "------------------------------------------\n");
  fprintf(fp_num_nodes_road, "------------------------------------------\n");

  //各パラメータの出力
  printf("r\t%f\nv\t%f\ndt\t%f\nD\t%f\nL\t%f\n", r, v, dt, D, L);
  fprintf(fp, "r\t%f\nv\t%f\ndt\t%f\nD\t%f\nL\t%f\n", r, v, dt, D, L);
  fprintf(fp_num_nodes_road, "r\t%f\nv\t%f\ndt\t%f\nD\t%f\nL\t%f\n", r, v, dt, D, L);
  printf("block_num\t%d\nA_i\t%f\nTwait_max\t%f\nSIM_NUM\t%d\n", block_num, A_i, Twait_max, SIM_NUM);
  fprintf(fp, "block_num\t%d\nA_i\t%f\nTwait_max\t%f\nSIM_NUM\t%d\n", block_num, A_i, Twait_max, SIM_NUM);
  fprintf(fp_num_nodes_road, "block_num\t%d\nA_i\t%f\nTwait_max\t%f\nSIM_NUM\t%d\n", block_num, A_i, Twait_max, SIM_NUM);

  printf("------------------------------------------\n");
  fprintf(fp, "------------------------------------------\n");
  fprintf(fp_num_nodes_road, "------------------------------------------\n");

  // 乱数の種を与える
  //Seed = (unsigned int)time(NULL);
  Seed = 111111;
  srand(Seed);

  printf("Seed\t%u\n", Seed);
  fprintf(fp, "Seed\t%u\n", Seed);
  fprintf(fp_num_nodes_road, "Seed\t%u\n", Seed);

  for (i = 0; i < ra_step_num; i++)
  {
    printf("p_avo[%d]\t%f\t", i, p_avo[i]);
    fprintf(fp, "p_avo[%d]\t%f\t", i, p_avo[i]);
    fprintf(fp_num_nodes_road, "p_avo[%d]\t%f\t", i, p_avo[i]);
  }
  printf("\n");
  fprintf(fp, "\n");
  fprintf(fp_num_nodes_road, "\n");

  printf("q_in\t%f\n", q);
  fprintf(fp, "q_in\t%f\n", q);
  fprintf(fp_num_nodes_road, "q_in\t%f\n", q);

  printf("D_TA\t%d", d_ta);
  fprintf(fp, "D_TA\t%d", d_ta);
  fprintf(fp_num_nodes_road, "D_TA\t%d", d_ta);

  for (i = 0; i < ra_step_num; i++)
  {
    printf("\tD_RA\t%d", d_ra[i]);
    fprintf(fp, "\tD_RA\t%d", d_ra[i]);
    fprintf(fp_num_nodes_road, "\tD_RA\t%d", d_ra[i]);
  }
  printf("\n");
  fprintf(fp, "\n");
  fprintf(fp_num_nodes_road, "\n");

  //カウンタ用の配列のメモリ動的確保
  Malloc_Counter();
  double sum_Twait = 0.0; //Twaitの合計値

  //カウンタの総和（全シミュレーション分）
  int sum_nodes_info[RA_STEP_NUM] = {0}; //情報を持ったノードの合計(0で初期化)
  int sum_nodes_no_info[1000]; //情報を持たないノードの合計
  int sum_not_having_passable_route[1000]; //もともとの経路ではRAに侵入するノードの合計
  for (i = 0; i < 1000; i++)
  { 
    sum_nodes_no_info[i] = 0;
    sum_not_having_passable_route[i] = 0;
  }
  
  int sum_recieved_unnecessary_info[RA_STEP_NUM] = {0}; //不必要な情報を持ったノードの合計
  int sum_recieved_necessary_info[RA_STEP_NUM] = {0}; //不必要でない情報を持ったノードの合計

  int sum_nodes_info_road[block_num * 2 + 1][block_num * 2 + 1];  //各道路区間に情報を持って存在するノードの合計
  int sum_nodes_no_info_road[block_num * 2 + 1][block_num * 2 + 1];  //各道路区間に情報を持たずに存在するノードの合計

  int sum_nodes_info_road2[4][block_num * 2 + 1][block_num * 2 + 1];
  int sum_nodes_no_info_road2[4][block_num * 2 + 1][block_num * 2 + 1];

  int sum_nodes_info_intersection[block_num + 1][block_num + 1];
  int sum_nodes_no_info_intersection[block_num + 1][block_num + 1];

  int sum_num_transmit_robot[1000] = {0};
  //ロボット用
  int sum_recieved_unnecessary_info_from_robot[RA_STEP_NUM] = {0}; //ロボットから不必要な情報を受け取った回数
  int sum_recieved_necessary_info_from_robot[RA_STEP_NUM] = {0};   //ロボットから必要な情報を受け取った回数

  //カウンタの初期化
  for (i = 0; i <= block_num; i++)
  {
    for (j = 0; j <= block_num; j++)
    {
      sum_nodes_info_intersection[i][j] = 0;
      sum_nodes_no_info_intersection[i][j] = 0;
    }
  }

  for (i = 0; i <= block_num * 2; i++)
  {
    for (j = 0; j <= block_num * 2; j++)
    {
      sum_nodes_info_road[i][j] = 0;
      sum_nodes_no_info_road[i][j] = 0;
      for (k = 0; k < 4; k++)
      {
        sum_nodes_info_road2[k][i][j] = 0;
        sum_nodes_no_info_road2[k][i][j] = 0;
      }
    }
  }

  //ロボット用の動的確保及びカウンタ初期化

  if (num_robot >= 1)
  {
    buf_robot_data = malloc(sizeof(double *) * (num_robot + 1));
    for (i = 0; i < (num_robot + 1); i++)
    {
      buf_robot_data[i] = malloc(sizeof(double) * 7);
    }
  }

  int count_node[RA_STEP_NUM] = {0}; //ノードの総数

  //進入するノードの密度はすべて同じ
  lambda_W = lambda;
  lambda_E = lambda;
  lambda_N = lambda;
  lambda_S = lambda;
  
  for (sim_time = 0; sim_time < SIM_NUM; sim_time++) //SIM_NUM回分for文を回す
  {

    //デフォルトとしてノードを消す
    Node_Clear(); 

    // FSを置いて，FSに情報を与える
    Dist(0, Ax / 2.0, Ay / 2.0, 0.0, 0.0, 0.0);
    Node[0].m_Info = TRUE;

    //一次元の場合のノードの初期配置 -> 空回しするので，初期配置はしない
    //Dist_Init();
    //Dist_Init2();

    //外から入ってきた時点で情報を持っているノードはいない
    Info_StartQ = FALSE;

    //送信可能エリアを設定
    Set_Transmittable_Area(); //送信可能範囲を設定する関数(Move.hで定義)
    //進入制限エリアを設定
    set_restricted_area(); //RAを1つ設定する関数(Move.hで定義)

    //ロボットを設置
    //ロボットの情報を記述したテキストファイルをrobot_dataフォルダにいれておくこと
    if (num_robot >= 1)
    {
      Get_Robot_Info(); //ロボットの情報をテキストデータから取得する関数(Node_Dist.h)
      for (i = 1; i <= num_robot; i++)
      {
        //二番目の引数に空回しでロボットを動かすなら1を、動かさないなら0を入れる(Node_Dist.h)
        Set_Robot(i, DO_ROBOT_EMPTY_TURN);
      }
    }

    
     

    

    for (i = 0; i < 100; i++)
    {
      Cross[i] = 0;
    }
 

    //空回し
    //初期化
    /////////////////////////////////////////////////////
    Twait = 0.0; //待ち時間を0に
    Sim_Flag = 0; //空回し終了フラグを0に(空回し未終了)

    //printf("空回しスタート\n");
    for (i = 0; i < RA_STEP_NUM; i++)
    {
      num_nodes_info[i] = 0;
      num_nodes_no_info[i] = 0;
      num_recieved_unnecessary_info[i] = 0;
      num_recieved_necessary_info[i] = 0;
    }

    for (i = 0; i <= block_num; i++)
    {
      for (j = 0; j <= block_num; j++)
      {
        num_nodes_info_intersection[i][j] = 0;
        num_nodes_no_info_intersection[i][j] = 0;
      }
    }

    for (i = 0; i <= block_num * 2; i++)
    {
      for (j = 0; j <= block_num * 2; j++)
      {
        num_nodes_info_road[i][j] = 0;
        num_nodes_no_info_road[i][j] = 0;
        for (k = 0; k < 4; k++)
        {
          num_nodes_info_road2[k][i][j] = 0;
          num_nodes_no_info_road2[k][i][j] = 0;
        }
      }
    }



    //ロボット関連の初期化
    for (i = 0; i < RA_STEP_NUM; i++)
    {
      num_recieved_unnecessary_info_from_robot[i] = 0;
      num_recieved_necessary_info_from_robot[i] = 0;
    }
    /////////////////////////////////////////////////////


    // 十分に長い時間空回し（行動変化も考慮すると，どれだけ回しても十分ではないかもしれないが）
    // while (Twait < (Ax + Ay) / v * 2.0 ) 
    // {
    //   //時間をdt進める
    //   Twait += dt;
    //   //ノードの追加配置
    //   Dist_Add();

    //   //ノードの移動
    //   move();

    //   //情報をフラッディング(情報を次々と広げる)する -> 空回しの段階でフラッディングもする
    //   flooding();

    // }
      //printf("空回し終了\n");

    //シミュレーション
    //初期化
    /////////////////////////////////////////////////////
    node_num = 0.0;

    Twait = 0.0;

    printf("シミュレーションスタート\n");
    
    for (i = 0; i < RA_STEP_NUM; i++)
    {
      num_nodes_info[i] = 0;
      num_nodes_no_info[i] = 0;
      num_recieved_unnecessary_info[i] = 0;
      num_recieved_necessary_info[i] = 0;
    }
    for (i = 0; i < 1000; i++)
    {
      num_not_having_passable_route[i] = 0;
      count_num_nodes_no_info[i] = 0; 
    }
    
    //ロボット関連
    for (i = 1; i <= num_robot; i++)
    {
      num_recieved_unnecessary_info_from_robot[i] = 0;
      num_recieved_necessary_info_from_robot[i] = 0;
      Node[i].m_Exist = 1;
    }
    /////////////////////////////////////////////////////
    Sim_Flag = 1; //空回し終了フラグ1(空回し終了済み)

    printf("lambda:%f\tsim_num:%d\n", lambda, sim_time);

    int print_count = 0;
    
    int Set_Robot_Flag = 0;
    int Delete_Robot_Flag = 0;
    // 観測時間の間，以下の処理を繰り返す
    while (Twait < Twait_max)
    {
      // if (Twait < 1)
      // {
      //   for (i = 1; i <= num_robot; i++)
      //   {
      //     Delete_Robot(i, DO_ROBOT_EMPTY_TURN);
      //   }
      // }

      // if (Twait > 3000 && Twait <= 4500 && Set_Robot_Flag == 0)
      // {
      //   if (num_robot >= 1)
      //   {
      //     Get_Robot_Info(); //ロボットの情報をテキストデータから取得する関数(Node_Dist.h)
      //     for (i = 1; i <= num_robot; i++)
      //     {
      //       //二番目の引数に空回しでロボットを動かすなら1を、動かさないなら0を入れる(Node_Dist.h)
      //       Set_Robot(i, DO_ROBOT_EMPTY_TURN);
      //     }
      //   }
      //   Set_Robot_Flag = 1;
      // }

      // if (Twait > 4500 && Delete_Robot_Flag == 0)
      // {
      //   for (i = 1; i <= num_robot; i++)
      //   {
      //     Delete_Robot(i, DO_ROBOT_EMPTY_TURN);
      //     Delete_Robot_Flag = 1;
      //   }
      // }
      
      //デバッグ用にノードの様子を画像ファイルに出力
      //動作が重くなり過ぎないように間引いて出力
      //カレントディレクトリにpngフォルダを作っておかないとエラーが出力される

      
    if (sim_time == 0)
    {
      if (PlotNodes_Flag == 1){
        if ((int)(Twait * 10 + 0.5) % 100 == 0)
         {
           PlotNodes();
        }
      }

    }
      

      //途中経過として時間を出力
      if ((int)(Twait * 10 + 0.5) % 100 == 0)
      {
        //printf("%f\n", Twait);
      }
      //時間をdt進める
      Twait += dt;
      
     
      
      
      //ノードの追加配置
      // if (Twait <= 1500)
      // {
      //     Dist_Add();
      // }
      
      //ノードの発生を時間変化させるために使用
      // if (Twait > 1500)
      // {
      //   Dist_Add2();
      // }
      Dist_Add2();

      //ノードの移動
      move();
      //情報をフラッディングする
      flooding();
      //各交差点付近の端末数をカウント
      Count_Num_Nodes_Intersection();
      //各道路の端末数をカウント
      Count_Num_Nodes_Road();

      if((int)(Twait * 10 + epsilon) % 1000 == 0) //100秒ごとにプリント
      {
        if(print_count == 0)
        {
          printf("\nnum not having passable route\n");
          fprintf(fp, "\nnum not having passable route\n");
          print_count++;
        }  
        printf("%.1f %d\n", Twait, num_not_having_passable_route[(int)((Twait + epsilon) / 100 ) -1]);
        fprintf(fp, "%7.1f-%7.1f %d\n", (double)i * 100, (double)((i + 1) * 100) - 1, num_not_having_passable_route[i]);
        sum_not_having_passable_route[(int)((Twait + epsilon) / 100) - 1] += num_not_having_passable_route[(int)((Twait + epsilon) / 100) - 1]; //時間毎にカウントしたものを時間毎に合計
      }
      
    } //観測時間の間，ここまでを繰り返す
    
    printf("Time, into RA\n");
    fprintf(fp, "Time, into RA\n");
    for (i = 0; i < Twait_max / 100; i++)
    {
      sum_nodes_no_info[i] += count_num_nodes_no_info[i]; //時間毎にカウントしたものを時間毎に合計
      sum_num_transmit_robot[i] += num_transmit_robot[i]; 
      printf("%7.1f-%7.1f %d\n", (double)i * 100, (double)( (i + 1) * 100 ) - 1, count_num_nodes_no_info[i]);
      fprintf(fp, "%7.1f-%7.1f %d\n", (double)i * 100, (double)( (i + 1) * 100 ) - 1, count_num_nodes_no_info[i]);
    }

    //各交差点からRAに進入するノード数
    for (i = 0; i < 12; i++)
    {
      //printf("Cross[%d] = %d\n", i, Cross[i]);
      TotalCross[i] += Cross[i];
    }
    
    for (i = 1; i <= num_robot; i++)
    {
      Delete_Robot(i, DO_ROBOT_EMPTY_TURN);
    }
    
    printf("%d回目のシミュレーション終了\n\n", sim_time + 1);
    fprintf(fp, "%d回目のシミュレーション終了\n\n", sim_time + 1);
    
    // シミュレーション一回分のカウントを加算
    sum_Twait += Twait;
    
    //メモリ解放
    if (num_robot != 3)
    {
      free(TA);
    }
    for (i = 0; i < ra_step_num + count_area_num - 1; i++)
    {  
      free(RA[i]);
    }
    free(RA);
  }
  //ここまでをSIM_NUM回繰り返す 

  printf("全てのシミュレーション終了\n");
  fprintf(fp, "全てのシミュレーション終了\n");
  
  //結果出力
  /////////////////////////////////////////////////////////////////////////////////
  printf("Seed\t%u\n", Seed);
  printf("lambda\t%f\n", lambda);
  fprintf(fp, "Seed\t%u\n", Seed);
  fprintf(fp, "lambda\t%f\n", lambda);
  
  printf("\ninto RA, num not having passable route\n");
  fprintf(fp, "\ninto RA, num not having passable route\n");
  for (i = 0; i < (Twait_max / 100) - 10 ; i++)
  {
    printf("%d %d\n", sum_nodes_no_info[i], sum_not_having_passable_route[i]);
    fprintf(fp, "%d %d\n", sum_nodes_no_info[i], sum_not_having_passable_route[i]);
    if (sum_nodes_no_info[i] > sum_not_having_passable_route[i])
    {
      printf("!!!ERROR!!!\n");
      break;
    }
  }
  
  if (num_robot != 0)
  {
    printf("\nロボットの情報送信回数\n");
    fprintf(fp, "\nロボットの情報送信回数\n");
    for (i = 0; i < (Twait_max / 100) - 10; i++)
    {
      printf("%d\n", sum_num_transmit_robot[i]);
      fprintf(fp, "%d\n", sum_num_transmit_robot[i]);
    }
  }
  
  printf("交差点へのノード進入数\n");
  for (i = 0; i < 12; i++)
  {
    printf("%d\n", TotalCross[i]);
    //printf("TotalCross[%d] = %d\n", i, TotalCross[i]);
  }
  
  // printf("count_LU = %d\n", count_LU);
  // printf("count_RU = %d\n", count_RU);
  // printf("count_LT = %d\n", count_LT);
  // printf("count_RT = %d\n", count_RT);
  // printf("Total = %d\n", count_LU + count_RU + count_LT + count_RT);
  /////////////////////////////////////////////////////////////////////////

  //ファイルポインタを閉じる
  fclose(fp);
  fclose(fp_num_nodes_road);

  return 0;
}
////////////////////////////////////////////
//コンパイルする際は-lmをつけるのを忘れないこと
////////////////////////////////////////////
