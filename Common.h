//////////////////////////////////////////////////
// 一般的な処理を行う関数群と定数，構造体の定義 //
//////////////////////////////////////////////////

////////////////
// 関数リスト //
//////////////////////////////////////////////////
// rand01()
// rand03()
// rand09()
// sqrt2()
// fact()
// Slope()
// Angle_Of_2Vector()
// d_Equal()
// Count_Having_Info_Nodes_Num_Into_RA()
// Count_Num_Nodes_Road()
// Count_Num_Nodes_Intersection()
// PlotNodes()
//////////////////////////////////////////////////

//各ノードの進路を指定するために使用
#define RIGHT 6
#define LEFT 4
#define UP 8
#define DOWN 2
#define EXIT 99

//各ノードが行動変化するかどうかを示す
#define AVOID 1
#define THROUGH -1

//ドローンの進路を指定する
#define UP_RIGHT 9
#define DOWN_RIGHT 3
#define UP_LEFT 7
#define DOWN_LEFT 1
#define STOP 5

//通常ノードを構造体で定義
typedef struct
{
  //共通で使うメンバ変数
  double m_X, m_Y;                         //ノードの位置
  double m_V;                              //ノードの絶対速度
  double m_Vx, m_Vy;                       //ノードの速度
  int m_Info;                              //情報を持っているか
  int m_Already_Flooding;                  //今の時間ステップにおいて、既にフラッディングしましたというフラグ
  int m_Exist;                             //ノードそのものが存在しているか
  double m_Goal_X, m_Goal_Y;               //目的地の座標
  int m_Avoid_Flag[RA_STEP_NUM];           //行動変化を示すフラグ(AVOID, THROUGH, FALSE)の3値をとる
  int m_Already_Set_Avoid_Flag;            //すでに迂回動作をしているかどうかを示す
  int m_Restricted_Area_Num;               //何番目のRAを迂回しているか
  int m_Received_From_FS;                  //FSから情報を受け取ったかどうか
  int m_Received_From_Robot;               //ロボットから情報を受け取ったかどうか
  int m_Already_Count_intoRA[RA_STEP_NUM]; //すでにRAに進入したノードとしてカウントしたかどうか
  int m_Already_Count_ruinfo;              //すでに不必要な情報を受け取ったかどうかについてカウントしたかどうか
  int m_Route[100];                        //経路を格納 とりあえず多めにメモリを確保しておく
  int m_Moved_num;                         //どこまで進んだかを示す（次にm_Route[]の何番目の要素を読むかを指定）
  int m_Enter_After_Idling;                //空回し終了後にサービスエリアに進入したかどうか
  int m_Robot;                             //ノードがロボットかどうか
  int m_ID;                                //ノードID
  double m_Time;                           //ノードが生成された時間
} SNode;

//このあとすぐ使うためノードをグローバル変数で定義しておく
SNode Node[N_MAX];

//ロボット用の情報を構造体で定義
typedef struct
{
  int m_Num_Route;                        //ロボットが使う経路の番号
  int m_Pause_Count;                      //ロボットが何回一時停止したか
  int m_On_Intersection;                  //ロボットが交差点上にいる時のフラグ
  double m_Pause_Remaining_Time;          //一時停止中のロボットの残り一時停止時間
  double m_Robot_Previous_Intersection_X; //ロボットが一つ前に通過した交差点のX座標
  double m_Robot_Previous_Intersection_Y; //ロボットが一つ前に通過した交差点のY座標
  double m_Transmission_distnace;         //ロボットの通信可能距離
} RNode;

RNode NodeR[R_MAX];

//ロボットの経路情報を構造体で定義
typedef struct
{
  int m_Route[R_SECTION_MAX];               //進行方向の情報
  double m_Section_Velocity[R_SECTION_MAX]; //各区間ごとの速度
  int m_Route_Len;                          //経路の区間数
  int m_Num_Pause_Point;                    //経路中にある停止位置の数
  double m_Pause_Time[R_SECTION_MAX];       //経路中の停止位置の停止時間
} SRoute;

SRoute Route[R_MAX];

//TA（送信可能エリア）の座標を格納するための構造体
//必ずm_X1 < m_X2, m_Y1 < m_Y2としておくこと
typedef struct
{
  double m_X1;
  double m_Y1;
  double m_X2;
  double m_Y2;
} Transmittable_Area;
//同様に定義しておく
Transmittable_Area *TA; //構造体へのポインタ

//RA（進入制限エリア）の座標を格納するための構造体
//必ずm_X1 < m_X2, m_Y1 < m_Y2としておくこと
typedef struct
{
  double m_X1;
  double m_Y1;
  double m_X2;
  double m_Y2;
} Restricted_Area;
Restricted_Area **RA;

//double型変数に対して等値かどうかを判定する関数
int d_Equal(double x, double y)
{
  if (fabs(x - y) < epsilon) //関数fabs: 絶対値を計算
    return 1;
  else
    return 0;
}

// (0,1)の一様乱数を返す
double rand01()
{
  int ran;

  //0が出たらやり直し
  do
  {
    ran = rand();
  } while (ran == 0);

  return (double)ran / RAND_MAX;
}

//0~3の整数をランダムに出力する
int rand03()
{
  return rand() % 4;
}

//0~9の整数をランダムに出力する
int rand09()
{
  return rand() % 10;
}

// 2乗和の平方根を返す
double sqrt2(double a_x, double a_y)
{
  return sqrt(a_x * a_x + a_y * a_y);
}

//階乗の計算をする関数
double fact(double x)
{
  if (d_Equal(x, 0) == 1)
    return 1;
  return (x * fact(x - 1));
}

//2点間を結ぶ直線の傾きを求める関数
double Slope(double x1, double y1, double x2, double y2)
{
  //x軸またはy軸に平行なときは0
  if (d_Equal(x1, x2) || d_Equal(y1, y2))
    return 0.0;

  return (y2 - y1) / (x2 - x1);
}

//(x0, y0)を原点とする2つのベクトル間の角度を0~M_PIで出力する
double Angle_Of_2Vector(double x0, double y0, double x1, double y1, double x2, double y2)
{
  double vec1[2], vec2[2];
  double cosine;
  vec1[0] = x1 - x0;
  vec1[1] = y1 - y0;

  vec2[0] = x2 - x0;
  vec2[1] = y2 - y0;

  cosine = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / sqrt2(vec1[0], vec1[1]) / sqrt2(vec2[0], vec2[1]);

  return acos(cosine);
}



//空回し終了後から情報を持って禁止エリアに入る端末と持たずに禁止エリアに入る端末の数を調べる
void Count_Having_Info_Nodes_Num_Into_RA(int ra_no, int a_i_node, double old_node_X, double old_node_Y, int route) //Move.hで使用
{
  // printf("Count_Having_Info_Nodes_Num_Into_RA\n");
  // printf("%d %d %3.0f %3.0f %d \n",ra_no,a_i_node,old_node_X,old_node_Y,route);
  int i, j;
  double delta = A_i / 2.0;

  if (Node[a_i_node].m_Enter_After_Idling == FALSE)  //m_Enter_After_Idlingは空回し終了後にサービスエリアに進入したかどうか
    return;

  if (route == RIGHT) //これから進む方向
  {
    for (i = 0; i < ra_num[ra_no]; i++)  //配列ra_numは進入制限エリアを表す構造体の数
    {
      if (old_node_X < RA[ra_no][i].m_X1 && RA[ra_no][i].m_X1 <= Node[a_i_node].m_X + delta && Node[a_i_node].m_X + delta <= RA[ra_no][i].m_X2 && RA[ra_no][i].m_Y1 <= Node[a_i_node].m_Y && Node[a_i_node].m_Y <= RA[ra_no][i].m_Y2)
      {
        Node[a_i_node].m_Already_Count_intoRA[ra_no] = TRUE; //すでにRAに進入したノードとしてカウントしたかどうか
        if (Node[a_i_node].m_Info == TRUE) //Nodeが情報を持っていたら
        {
           num_nodes_info[ra_no]++; //情報を持ってRAに進入したノードの数をインクリメント
        }  
        else
        {
          num_nodes_no_info[ra_no]++;  //情報を持たずにRAに進入したノードの数をインクリメント
          
          if (i == 5)
          {
            Cross[9]++;
          }

          if (i == 6)
          {
            Cross[10]++;
          }

          if (i == 7)
          {
            Cross[11]++;
          }
          
          if (i == 8)
          {
            Cross[8]++;
          }

          if (i == 9)
          {
            Cross[7]++;
          }

          //printf("Cross[0] = %d\n", Cross[0]);
          //printf("RIGHT RA[%d]\n", i);
        }
        
        for (j = 0; j < Twait_max / 100; j++)
        {
          if (Node[a_i_node].m_Info == FALSE && j * 100 < Node[a_i_node].m_Time && Node[a_i_node].m_Time <= (j + 1) * 100)
          {
            count_num_nodes_no_info[j]++; //生起した時間毎に，情報を持たずにRAに進入したノードの数をインクリメント
            
            //printf("R %d, %f\n", Node[a_i_node].m_ID, Twait);
            //printf("R %d, %f\n", Node[a_i_node].m_ID, Node[a_i_node].m_Time);
          }
        }
        return;
      }
    }
  }
  if (route == LEFT)
  {
    for (i = 0; i < ra_num[ra_no]; i++)
    {
      if (old_node_X > RA[ra_no][i].m_X2 && RA[ra_no][i].m_X1 <= Node[a_i_node].m_X - delta && Node[a_i_node].m_X - delta <= RA[ra_no][i].m_X2 && RA[ra_no][i].m_Y1 <= Node[a_i_node].m_Y && Node[a_i_node].m_Y <= RA[ra_no][i].m_Y2)
      {
        Node[a_i_node].m_Already_Count_intoRA[ra_no] = TRUE;
        if (Node[a_i_node].m_Info == TRUE)
        {
          num_nodes_info[ra_no]++;
        }
        else
        {
          
          num_nodes_no_info[ra_no]++;  //情報を持たずにRAに進入したノードの数をインクリメント
          //printf("i = %d\n", i);
          if (i == 5)
          {
            Cross[3]++;
          }

          if (i == 6)
          {
            Cross[2]++;
          }

          if (i == 7)
          {
            Cross[1]++;
          }
          
          if (i == 8)
          {
            Cross[4]++;
          }

          if (i == 9)
          {
            Cross[5]++;
          }
          //printf("LEFT RA[%d]\n", i);
        }

        for (j = 0; j < Twait_max / 100; j++)
        {
          if (Node[a_i_node].m_Info == FALSE && j * 100 < Node[a_i_node].m_Time && Node[a_i_node].m_Time <= (j + 1) * 100)
          {
            count_num_nodes_no_info[j]++; 
            
            //printf("L %d, %f\n", Node[a_i_node].m_ID, Twait);
            //printf("L %d, %f\n", Node[a_i_node].m_ID, Node[a_i_node].m_Time);
          }
        }
          
        return;
      }
    }
  }
  if (route == UP)
  {
    for (i = 0; i < ra_num[ra_no]; i++)
    {
      if (old_node_Y < RA[ra_no][i].m_Y1 && RA[ra_no][i].m_X1 <= Node[a_i_node].m_X && Node[a_i_node].m_X <= RA[ra_no][i].m_X2 && RA[ra_no][i].m_Y1 <= Node[a_i_node].m_Y + delta && Node[a_i_node].m_Y + delta <= RA[ra_no][i].m_Y2)
      {
        Node[a_i_node].m_Already_Count_intoRA[ra_no] = TRUE;
        if (Node[a_i_node].m_Info == TRUE)
          num_nodes_info[ra_no]++;
       else
        {
          num_nodes_no_info[ra_no]++;  //情報を持たずにRAに進入したノードの数をインクリメント
          // printf("i = %d\n", i);
          if (i == 0)
          {
            Cross[6]++;
          }

          if (i == 1)
          {
            Cross[5]++;
          }

          if (i == 2)
          {
            Cross[4]++;
          }
          
          if (i == 3)
          {
            Cross[7]++;
          }

          if (i == 4)
          {
            Cross[8]++;
          }
          //printf("UP RA[%d]\n", i);
        }

        for (j = 0; j < Twait_max / 100; j++)
        {
          if (Node[a_i_node].m_Info == FALSE && j * 100 < Node[a_i_node].m_Time && Node[a_i_node].m_Time <= (j + 1) * 100)
          {
            count_num_nodes_no_info[j]++;
            //printf("U %d, %f\n", Node[a_i_node].m_ID, Twait); 
            //printf("U %d, %f\n", Node[a_i_node].m_ID, Node[a_i_node].m_Time);
          }
        }
          
        return;
      }
    }
  }

  if (route == DOWN)
  {
    for (i = 0; i < ra_num[ra_no]; i++)
    {
      if (old_node_Y > RA[ra_no][i].m_Y2 && RA[ra_no][i].m_X1 <= Node[a_i_node].m_X && Node[a_i_node].m_X <= RA[ra_no][i].m_X2 && RA[ra_no][i].m_Y1 <= Node[a_i_node].m_Y - delta && Node[a_i_node].m_Y - delta <= RA[ra_no][i].m_Y2)
      {
        Node[a_i_node].m_Already_Count_intoRA[ra_no] = TRUE;
        if (Node[a_i_node].m_Info == TRUE)
          num_nodes_info[ra_no]++;
        else
        {
          num_nodes_no_info[ra_no]++;  //情報を持たずにRAに進入したノードの数をインクリメント
           if (i == 0)
          {
            Cross[0]++;
          }

          if (i == 1)
          {
            Cross[1]++;
          }

          if (i == 2)
          {
            Cross[2]++;
          }
          
          if (i == 3)
          {
            Cross[11]++;
          }

          if (i == 4)
          {
            Cross[10]++;
          }
          //printf("DOWN RA[%d]\n", i);
        }

        for (j = 0; j < Twait_max / 100; j++)
        {
          if (Node[a_i_node].m_Info == FALSE && j * 100 < Node[a_i_node].m_Time && Node[a_i_node].m_Time <= (j + 1) * 100)
          {
            count_num_nodes_no_info[j]++; 
            //printf("D %d, %f\n", Node[a_i_node].m_ID, Twait);
            //printf("D %d, %f\n", Node[a_i_node].m_ID, Node[a_i_node].m_Time);

          }
        }
        return;
      }
    }
  }
}

//各道路区間に存在するノード数をカウント(交差点以外)
void Count_Num_Nodes_Road() //mainで使用
{
  int i_node, i_x, i_y;
  int counted_flag;

  for (i_node = 1; i_node < N_MAX; i_node++)
  {
    counted_flag = FALSE; //カウントフラグをFALSEに

    if (Node[i_node].m_Exist == FALSE) //ノードが存在していない場合continue
      continue;

    //水平方向に伸びる道路上にいるかどうか
    for (i_y = 0; i_y < block_num + 1; i_y++)
    {
      for (i_x = 0; i_x < block_num; i_x++)
      {

        if (d_Equal(Node[i_node].m_Y, A_i * i_y) && Node[i_node].m_X >= A_i * i_x && Node[i_node].m_X < A_i * (i_x + 1)) //ノードのy座標 == (交差点の区間の長さ * カウンタ) かつ，ノードのx座標がある交差点間にあるなら
        {
          //情報を持つ
          if (Node[i_node].m_Info == TRUE)
          {
            num_nodes_info_road[i_y * 2][i_x * 2 + 1]++; //各道路区間に存在するノード数をインクリメント
            if (Node[i_node].m_Vx > 0.0) //ノードのx方向の速度が正 --> 右に進む
              num_nodes_info_road2[0][i_y * 2][i_x * 2 + 1]++; //[0]は右に進む(?) ノード数をインクリメント
            if (Node[i_node].m_Vx < 0.0) //ノードのx方向の速度が負 --> 左に進む
              num_nodes_info_road2[1][i_y * 2][i_x * 2 + 1]++;  //[1]は左に進む(?) ノード数をインクリメント
            counted_flag = TRUE; //カウントフラグをTRUEに
            break;
          }
          //情報を持たない
          if (Node[i_node].m_Info == FALSE)
          {
            num_nodes_no_info_road[i_y * 2][i_x * 2 + 1]++;
            if (Node[i_node].m_Vx > 0.0)
              num_nodes_no_info_road2[0][i_y * 2][i_x * 2 + 1]++;
            if (Node[i_node].m_Vx < 0.0)
              num_nodes_no_info_road2[1][i_y * 2][i_x * 2 + 1]++;
            counted_flag = TRUE;
            break;
          }
        }
      }
      if (counted_flag == TRUE)
        break;
    }
    //すでに数えていたら次のノードを見る
    if (counted_flag == TRUE)
      continue;

    //垂直方向に伸びる道路上にいるかどうか
    for (i_y = 0; i_y < block_num; i_y++)
    {
      for (i_x = 0; i_x < block_num + 1; i_x++)
      {

        if (d_Equal(Node[i_node].m_X, A_i * i_x) &&
            Node[i_node].m_Y >= A_i * i_y && Node[i_node].m_Y < A_i * (i_y + 1))
        {
          //情報を持つ
          if (Node[i_node].m_Info == TRUE)
          {
            num_nodes_info_road[i_y * 2 + 1][i_x * 2]++;
            if (Node[i_node].m_Vy > 0.0)
              num_nodes_info_road2[2][i_y * 2 + 1][i_x * 2]++;
            if (Node[i_node].m_Vy < 0.0)
              num_nodes_info_road2[3][i_y * 2 + 1][i_x * 2]++;
            counted_flag = TRUE;
            break;
          }
          //情報を持たない
          if (Node[i_node].m_Info == FALSE)
          {
            num_nodes_no_info_road[i_y * 2 + 1][i_x * 2]++;
            if (Node[i_node].m_Vy > 0.0)
              num_nodes_no_info_road2[2][i_y * 2 + 1][i_x * 2]++;
            if (Node[i_node].m_Vy < 0.0)
              num_nodes_no_info_road2[3][i_y * 2 + 1][i_x * 2]++;
            counted_flag = TRUE;
            break;
          }
        }
      }
      if (counted_flag == TRUE)
        break;
    }
  }
  return;
}

//各交差点付近に存在するノード数のカウント
void Count_Num_Nodes_Intersection()
{
  int i, j, i_node, counted_flag;

  for (i_node = 1; i_node < N_MAX; i_node++)
  {
    counted_flag = FALSE;
    if (Node[i_node].m_Exist == FALSE)
      continue;
    for (i = 0; i <= block_num; i++)
    {
      for (j = 0; j <= block_num; j++)
      {
        if (Node[i_node].m_X >= A_i * (i - 0.5) && Node[i_node].m_X < A_i * (i + 0.5) && Node[i_node].m_Y >= A_i * (j - 0.5) && Node[i_node].m_Y < A_i * (j + 0.5))
        {
          if (Node[i_node].m_Info == TRUE)
            num_nodes_info_intersection[i][j]++;
          if (Node[i_node].m_Info == FALSE)
            num_nodes_no_info_intersection[i][j]++;
          counted_flag = TRUE;
          break;
        }
        if (counted_flag == TRUE)
          break;
      }
    }
  }
  return;
}

//カウンタ用変数のメモリ確保を行う関数
void Malloc_Counter()
{
  int i, j;

  if ((num_nodes_info_road = (int **)malloc(sizeof(int *) * (block_num * 2 + 1))) == NULL)
  {
    puts("memory allocation error ( num_nodes_info_road )");
    exit(EXIT_FAILURE);
  }
  if ((num_nodes_no_info_road = (int **)malloc(sizeof(int *) * (block_num * 2 + 1))) == NULL)
  {
    puts("memory allocation error ( num_nodes_no_info_road )");
    exit(EXIT_FAILURE);
  }

  for (i = 0; i < block_num * 2 + 1; i++)
  {
    if ((num_nodes_info_road[i] = (int *)malloc(sizeof(int) * (block_num * 2 + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_info_road[] )");
      exit(EXIT_FAILURE);
    }
    if ((num_nodes_no_info_road[i] = (int *)malloc(sizeof(int) * (block_num * 2 + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_no_info_road[] )");
      exit(EXIT_FAILURE);
    }
  }

  for (i = 0; i < 4; i++)
  {
    if ((num_nodes_info_road2[i] = (int **)malloc(sizeof(int *) * (block_num * 2 + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_info_road2[] )");
      exit(EXIT_FAILURE);
    }
    if ((num_nodes_no_info_road2[i] = (int **)malloc(sizeof(int *) * (block_num * 2 + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_no_info_road2[] )");
      exit(EXIT_FAILURE);
    }
  }

  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < block_num * 2 + 1; j++)
    {
      if ((num_nodes_info_road2[i][j] = (int *)malloc(sizeof(int) * (block_num * 2 + 1))) == NULL)
      {
        puts("memory allocation error ( num_nodes_info_road2[][] )");
        exit(EXIT_FAILURE);
      }
      if ((num_nodes_no_info_road2[i][j] = (int *)malloc(sizeof(int) * (block_num * 2 + 1))) == NULL)
      {
        puts("memory allocation error ( num_nodes_no_info_road2[][] )");
        exit(EXIT_FAILURE);
      }
    }
  }

  if ((num_nodes_info_intersection = (int **)malloc(sizeof(int *) * (block_num + 1))) == NULL)
  {
    puts("memory allocation error ( num_nodes_info_intersection )");
    exit(EXIT_FAILURE);
  }
  if ((num_nodes_no_info_intersection = (int **)malloc(sizeof(int *) * (block_num + 1))) == NULL)
  {
    puts("memory allocation error ( num_nodes_no_info_intersection )");
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < block_num + 1; i++)
  {
    if ((num_nodes_info_intersection[i] = (int *)malloc(sizeof(int) * (block_num + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_info_intersection[] )");
      exit(EXIT_FAILURE);
    }
    if ((num_nodes_no_info_intersection[i] = (int *)malloc(sizeof(int) * (block_num + 1))) == NULL)
    {
      puts("memory allocation error ( num_nodes_no_info_intersection[] )");
      exit(EXIT_FAILURE);
    }
  }
}

//gnuplotにパイプを通してノードの様子を画像ファイルに出力する関数
//画像ファイルはカレントディレクトリ内のpngフォルダに保存する
//移動のデバッグ用
void PlotNodes()
{
  FILE *gp;
  int i, i_ra_no;

  gp = popen("gnuplot -persist", "w");
  fprintf(gp, "set datafile separator ','\n");
  fprintf(gp, "set xrange [%f:%f]\n", -v * dt - 500, Ax + v * dt + 500);
  fprintf(gp, "set yrange [%f:%f]\n", -v * dt - 500, Ay + v * dt + 500);
  fprintf(gp, "set xtics %d\n", (int)A_i * 2);
  fprintf(gp, "set mxtics %d\n", 2);
  fprintf(gp, "set ytics %d\n", (int)A_i * 2);
  fprintf(gp, "set mytics %d\n", 2);
  fprintf(gp, "set grid xtics mxtics ytics mytics\n");
  fprintf(gp, "set size square\n");
  fprintf(gp, "set terminal png\n");
  fprintf(gp, "set output './png/%f.png'\n", Twait);

  //TAを描画
  for (i = 0; i < ta_num; i++)
  {
    fprintf(gp, "set arrow from %f, %f to %f, %f  lw 2 nohead\n",
            TA[i].m_X1, TA[i].m_Y1, TA[i].m_X2, TA[i].m_Y1);
    fprintf(gp, "set arrow from %f, %f to %f, %f  lw 2 nohead\n",
            TA[i].m_X2, TA[i].m_Y1, TA[i].m_X2, TA[i].m_Y2);
    fprintf(gp, "set arrow from %f, %f to %f, %f  lw 2 nohead\n",
            TA[i].m_X2, TA[i].m_Y2, TA[i].m_X1, TA[i].m_Y2);
    fprintf(gp, "set arrow from %f, %f to %f, %f  lw 2 nohead\n",
            TA[i].m_X1, TA[i].m_Y2, TA[i].m_X1, TA[i].m_Y1);
  }
  //RAを描画
  for (i_ra_no = ra_step_num + 3; i_ra_no >= 0; i_ra_no--)
  {
    for (i = 0; i < ra_num[i_ra_no]; i++)
    {
      fprintf(gp, "set arrow from %f, %f to %f, %f  lw 1 lt %d nohead\n",
              RA[i_ra_no][i].m_X1, RA[i_ra_no][i].m_Y1, RA[i_ra_no][i].m_X2, RA[i_ra_no][i].m_Y1, i_ra_no + 1);
      fprintf(gp, "set arrow from %f, %f to %f, %f  lw 1 lt %d nohead\n",
              RA[i_ra_no][i].m_X2, RA[i_ra_no][i].m_Y1, RA[i_ra_no][i].m_X2, RA[i_ra_no][i].m_Y2, i_ra_no + 1);
      fprintf(gp, "set arrow from %f, %f to %f, %f  lw 1 lt %d nohead\n",
              RA[i_ra_no][i].m_X2, RA[i_ra_no][i].m_Y2, RA[i_ra_no][i].m_X1, RA[i_ra_no][i].m_Y2, i_ra_no + 1);
      fprintf(gp, "set arrow from %f, %f to %f, %f  lw 1 lt %d nohead\n",
              RA[i_ra_no][i].m_X1, RA[i_ra_no][i].m_Y2, RA[i_ra_no][i].m_X1, RA[i_ra_no][i].m_Y1, i_ra_no + 1);
    }
  }

  //各ノードの現在位置から目的地に向かって矢印を描画
  /*
  for ( i = 1; i < N_MAX; i ++ ){
    if( Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE && Node[i].m_Received_From_FS == FALSE )
          fprintf(gp, "set arrow from %f, %f to %f, %f  lw 2\n",
                  Node[i].m_X, Node[i].m_Y, Node[i].m_Goal_X, Node[i].m_Goal_Y );
  }
  */

   //ノードを識別するためのラベル
  // for ( i = 1; i < N_MAX; i++ ){
  //   if ( Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE )
  //     fprintf(gp, "set label '%d' at %f, %f\n", Node[i].m_ID, Node[i].m_X, Node[i].m_Y);

  //   if ( Node[i].m_Info == FALSE && Node[i].m_Exist == TRUE )
  //     fprintf(gp, "set label '%d' at %f, %f\n", Node[i].m_ID, Node[i].m_X, Node[i].m_Y);
  // }

  //ノードの描画
  fprintf(gp, "plot '-' w p pt 7 ps 1 lc rgb 'royalblue' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 1 lc rgb 'purple' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 1 lc rgb 'orange' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 1 lc rgb 'brown' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 1 lc rgb 'red' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 2 lc rgb 'dark-violet' notitle,");
  fprintf(gp, "'-' w p pt 7 ps 1 lc rgb 'green' notitle\n");


  //情報を持っていないノード royalblue
  for (i = 0; i < N_MAX; i++)
  {
    if (Node[i].m_Info == FALSE && Node[i].m_Exist == TRUE){
      fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
  }
  }
  fprintf(gp, "e\n");

  //ロボットから情報をもらったノード purple
  for (i = 0; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      
      if (Node[i].m_Received_From_FS == FALSE && Node[i].m_Restricted_Area_Num < 0 && Node[i].m_Received_From_Robot == 1)
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  //FSから情報をもらったノード orange
  for (i = 1; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      if (Node[i].m_Received_From_FS == TRUE && Node[i].m_Restricted_Area_Num < 0 )
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  //通常ノードから情報をもらったノード brown
  for (i = 1; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      if (Node[i].m_Received_From_FS == FALSE && Node[i].m_Restricted_Area_Num < 0 && Node[i].m_Received_From_Robot == 0)
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  //RAを迂回中のノード red
  for (i = 1; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      if (Node[i].m_Received_From_FS == FALSE && Node[i].m_Restricted_Area_Num == 0)
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  //ロボット dark-violet
  for (i = 1; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      if (i <= num_robot)
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  //情報を持ってRAに入るノード(いない) かつ ロボットからも情報を受け取らない green
  for (i = 1; i < N_MAX; i++)
  {
    if (Node[i].m_Info == TRUE && Node[i].m_Exist == TRUE)
    {
      if (Node[i].m_Received_From_FS == FALSE && Node[i].m_Restricted_Area_Num == 1)
        fprintf(gp, "%f, %f\n", Node[i].m_X, Node[i].m_Y);
    }
  }
  fprintf(gp, "e\n");

  pclose(gp);
}

