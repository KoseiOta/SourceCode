///////////////////////////////
//ノードの配置に関する関数群 //
///////////////////////////////

////////////////
// 関数リスト //
//////////////////////////////////////////////////
// Node_Clear()
// Dist()
// Seg_Length()
// Dist_in_Interval()
// Dist_in_Interval2()
// move()
// Dist_Add()
// Dist_Add2()
// Get_Robot_Info()
// Set_Robot()
// Delete_Robot()
//////////////////////////////////////////////////

//グローバル変数 各方角から発生したノードの個数を調べるために使用
int count_S = 0;
int count_N = 0;
int count_W = 0;
int count_E = 0;

//ノードをすべて消去する関数
void Node_Clear() //mainで使用
{
  int i;

  //ノードの存在を消す
  for (i = 0; i < N_MAX; i++)
  {
    Node[i].m_Exist = FALSE;
  }

  ID = 0; //次に発生するノードのノードID
}

//位置(a_x, a_y)に配置して、速度を(a_vx, a_vy)とする
void Dist(int a_i_node, double a_x, double a_y, double a_v, double a_vx, double a_vy) //main, Dist_in_Intervalで使用
{
  int i;
  
  Node[a_i_node].m_X = a_x; //位置を(a_x, a_y)とする
  Node[a_i_node].m_Y = a_y;

  // if(d_Equal(Node[a_i_node].m_Y, 0) == 1 && Sim_Flag == 1)
  // {
  //   count_S++;
  // }
  
  // if(d_Equal(Node[a_i_node].m_X, 0) == 1 && Sim_Flag == 1)
  // {
  //   count_W++;
  // }

  // if(d_Equal(Node[a_i_node].m_Y, 6000) == 1 && Sim_Flag == 1)
  // {
  //   count_N++;
  // }

  // if(d_Equal(Node[a_i_node].m_X, 6000) == 1 && Sim_Flag == 1)
  // {
  //   count_E++;
  // }
 
  Node[a_i_node].m_V = a_v; //絶対速度を与える

  Node[a_i_node].m_Vx = a_vx; //速度を(a_vx, a_vy)とする
  Node[a_i_node].m_Vy = a_vy;

  Node[a_i_node].m_Info = FALSE; //配置時に情報は持っていない

  Node[a_i_node].m_Exist = TRUE;  //ノードが存在することになる
  Node[a_i_node].m_Robot = FALSE; //通常ノードなのでロボットではない

  Node[a_i_node].m_Received_From_Robot = FALSE;  
  //各種フラグ初期化
  for (i = 0; i < ra_step_num; i++)
    Node[a_i_node].m_Avoid_Flag[i] = FALSE; //行動変化を示すフラグ

  Node[a_i_node].m_Restricted_Area_Num = -1; //何番目のRAを迂回しているか

  Node[a_i_node].m_Already_Set_Avoid_Flag = FALSE; //既に迂回動作をしているか

  Node[a_i_node].m_Received_From_FS = FALSE; //FSから情報を受け取ったかどうか
  for (i = 0; i < RA_STEP_NUM; i++)
    Node[a_i_node].m_Already_Count_intoRA[i] = FALSE; //既にRAに進入したノードとしてカウントしたかどうか

  Node[a_i_node].m_Already_Count_ruinfo = FALSE; //既に不必要な情報について受け取ったかどうかについてカウントしたかどうか

  
  Node[a_i_node].m_Time = Twait; //ノードが生成された時間を格納
  if (Sim_Flag == 1) //空回し終了フラグ 1は空回し終了済
  {  
    Node[a_i_node].m_Enter_After_Idling = TRUE; //空回し終了後にSAに進入したかどうか
    //printf("x = %d, y = %d\n", (int)(Node[a_i_node].m_X), (int)(Node[a_i_node].m_Y));
    
    if((int)(Node[a_i_node].m_Y) == 0 && Sim_Flag == 1)
    {
      count_S++;
    }
  
    if((int)(Node[a_i_node].m_X) == 0 && Sim_Flag == 1)
    {
      count_W++;
    }

    if((int)(Node[a_i_node].m_Y) == 6000 && Sim_Flag == 1)
    {
      count_N++;
    }

    if((int)(Node[a_i_node].m_X) == 6000 && Sim_Flag == 1)
    {
      count_E++;
    }
  
  }  
  else
    Node[a_i_node].m_Enter_After_Idling = FALSE;

  Node[a_i_node].m_ID = ID; //ノードID
  //printf("ID = %d\n", Node[a_i_node].m_ID);
  ID++;

  // if (Node[a_i_node].m_ID == 47)
  //    Node[a_i_node].m_Exist = FALSE;
}

//密度を与えて、セグメント長を返す 
double Seg_Length(double a_lambda)
{
  return -log(rand01()) / a_lambda;
}

//(a_x1, a_y1)から(a_x2, a_y2)までの線分上にノードを配置して、速度を(a_vx, a_vy)とする
//配置間隔は密度a_lambdaの指数分布に従う
void Dist_in_Interval(double a_x1, double a_y1, double a_x2, double a_y2, double a_vx, double a_vy, double a_lambda, int a_Info)
{
  int i, j, i_ra_no; //i_ra_noはra_noのi版

  
  //端末密度が0以下だったら、端末を配置しない
  if (a_lambda <= 0.0)
    return;

  double Length = sqrt2(a_x2 - a_x1, a_y2 - a_y1); // (a_x1, a_y1)と(a_x2, a_y2)との間の距離
  double segment;
  double sum_segment = 0.0;
  //無限ループ
  while (TRUE)
  {
    //密度a_lambdaの指数乱数に従うsegmentを計算
    segment = Seg_Length(a_lambda);
    sum_segment += segment;

    //sum_segmentがLength以下だったら、ノードを配置
    if (sum_segment <= Length)
    {
      double p = sum_segment / Length; //セグメント長
      double x = a_x1 * (1 - p) + a_x2 * p;
      double y = a_y1 * (1 - p) + a_y2 * p;

      //ノードが存在していないメモリを検索し，見つけたらそこに格納
      for (i = 0; i < N_MAX; i++)
      {
        if (Node[i].m_Exist == FALSE && Node[i].m_Robot == FALSE) //ノードが存在しておらず，ロボットでもない
        {
          Dist(i, x, y, v, a_vx, a_vy); //ノードを位置(x, y)に配置して、速度を(a_vx, a_vy)とする
          Set_Goal(i);  //サービスエリアから出る座標を決定する
          Set_Route(i); //サービスエリアを出るまでの経路を決定する

          if (Have_Passable_Route(i, 0) == FALSE && Node[i].m_Exist == TRUE) //存在するノードiの生成時の目的地までの経路がRAを通る
          {
            num_not_having_passable_route[ (int)(Node[i].m_Time / 100)]++;
          }

          if (a_Info == TRUE) //情報を持つなら
            Node[i].m_Info = TRUE; //情報を持つとする
          
          break;
        }

        if (i == N_MAX - 1)
        {
          printf("Too many nodes exist.\n");
          exit(-1);
        }
      }
    }
    else
      break;
  }
}

void Dist_in_Interval2(double a_x1, double a_y1, double a_x2, double a_y2, double a_vx, double a_vy, double a_lambda, int a_Info)
{
  int i, j, i_ra_no; //i_ra_noはra_noのi版

  
  //端末密度が0以下だったら、端末を配置しない
  if (a_lambda <= 0.0)
    return;

  double Length = sqrt2(a_x2 - a_x1, a_y2 - a_y1); // (a_x1, a_y1)と(a_x2, a_y2)との間の距離
  double segment;
  double sum_segment = 0.0;
  //無限ループ
  while (TRUE)
  {
    //密度a_lambdaの指数乱数に従うsegmentを計算
    segment = Seg_Length(a_lambda);
    sum_segment += segment;

    //sum_segmentがLength以下だったら、ノードを配置
    if (sum_segment <= Length)
    {
      double p = sum_segment / Length; //セグメント長
      double x = a_x1 * (1 - p) + a_x2 * p;
      double y = a_y1 * (1 - p) + a_y2 * p;

      //ノードが存在していないメモリを検索し，見つけたらそこに格納
      for (i = 0; i < N_MAX; i++)
      {
        if (Node[i].m_Exist == FALSE && Node[i].m_Robot == FALSE) //ノードが存在しておらず，ロボットでもない
        {
          Dist(i, x, y, v, a_vx, a_vy); //ノードを位置(x, y)に配置して、速度を(a_vx, a_vy)とする
          Set_Goal2(i);  //サービスエリアから出る座標を決定する
          Set_Route(i); //サービスエリアを出るまでの経路を決定する

          if (Have_Passable_Route(i, 0) == FALSE && Node[i].m_Exist == TRUE) //存在するノードiの生成時の目的地までの経路がRAを通る
          {
            num_not_having_passable_route[ (int)(Node[i].m_Time / 100)]++;
          }

          if (a_Info == TRUE) //情報を持つなら
            Node[i].m_Info = TRUE; //情報を持つとする
          
          break;
        }

        if (i == N_MAX - 1)
        {
          printf("Too many nodes exist.\n");
          exit(-1);
        }
      }
    }
    else
      break;
  }
}

//存在するすべてのノードiを移動させる関数
void move()
{
  int i;

  for (i = 0; i < N_MAX; i++)
  {
    //ノードが存在しない場合はcontinue
    if (Node[i].m_Exist == FALSE)
      continue;
    //ノードを移動
    if (Node[i].m_Robot == FALSE)
    {
      Move(i); //ロボットでなければノードをmove
    }
    else
    {
      Move_Robot(i); //ロボットであればロボットをmove
    }
  }
}

//ノードの追加配置
void Dist_Add() //mainで2度使用
{
  //移動ノードを追加配置
  int i = 0;

  for (i = 0; i <= block_num; i++)
  {
    Dist_in_Interval(0.0, A_i * i, -v * dt, A_i * i, v, 0.0, lambda_W, FALSE); //Wから生成
    Dist_in_Interval(Ax, A_i * i, Ax + v * dt, A_i * i, -v, 0.0, lambda_E, FALSE); //E
  }
  for (i = 0; i <= block_num; i++)
  {
    Dist_in_Interval(A_i * i, 0.0, A_i * i, -v * dt, 0.0, v, lambda_S, FALSE); //S
    Dist_in_Interval(A_i * i, Ay, A_i * i, Ay + v * dt, 0.0, -v, lambda_N, FALSE); //N
  }
  
}


void Dist_Add2() 
{
  //移動ノードを追加配置
  int i = 0;

  for (i = 0; i <= block_num; i++)
  {
    // if (i >= 2 && i <= 10)
    // {
    //   continue;
    // }
    Dist_in_Interval2(0.0, A_i * i, -v * dt, A_i * i, v, 0.0, lambda_W, FALSE); //Wから生成
  }
  for (i = 0; i <= block_num; i++)  
  {
  //   if (i >= 2 && i <= 10)
  //   {
  //     continue;
  //   }

    Dist_in_Interval2(Ax, A_i * i, Ax + v * dt, A_i * i, -v, 0.0, lambda_E, FALSE); //E
  }
  
  for (i = 0; i <= block_num; i++)
  {
    
    // if (i >= 2 && i <= 10)
    // {
    //   continue;
    // }
    Dist_in_Interval2(A_i * i, 0.0, A_i * i, -v * dt, 0.0, v, lambda_S, FALSE); //S
  }
  for (i = 0; i <= block_num; i++)  
  {
    
    // if (i >= 2 && i <= 10)
    // {
    //   continue;
    // }
    Dist_in_Interval2(A_i * i, Ay, A_i * i, Ay + v * dt, 0.0, -v, lambda_N, FALSE); //N
  }
}

//ロボットの情報をテキストデータから取得する関数
void Get_Robot_Info()
{
  FILE *fp; // FILE型構造体
  char filename[40];
  char path[1024];
  char line[1024];
  char *buf;
  char *p;
  

  int i = 1;
  int j = 0;
  int get_route_num = 1;     //読み込む経路数
  int count_pause_point = 0; //一時停止位置のカウント
  int route_num;             //経路の番号  

  //ファイル名
  sprintf(filename, "robotdata_c%dn%d.txt", robot_condition_num, num_robot); //robot_conditon_num:条件番号, num_robot:ロボット数
  //ディレクトリ
  sprintf(path, "robot_data/%s", filename);

  fp = fopen(path, "r"); // ファイルを開く. 失敗するとNULLを返す.
  if (fp == NULL)
  {
    printf("%s file not open!\n", filename);
    exit(1);
  }

  //テキストデータの内容の格納
  while ((fgets(line, 256, fp)) != NULL)
  {
    //ロボットの各種データを格納
    //前から順に初期x座標,初期y座標,経路の番号,経路の何番目から読み始めるか,最初に情報を持つかどうか,飛行高度,通信可能距離
    if (i <= num_robot)
    {
      //ロボットの各種データを格納
      p = line;
      while (1)
      {
        buf = strtok(p, " ");
        if (buf == NULL)
          break;
        buf_robot_data[i][j] = atof(buf); //buf_robot_data:ロボットのデータの一時格納, 関数atoi:文字列で表現された数値をint型の数値に変換する。
        printf("buf_robot_data[%d][%d] = %f\n", i, j, buf_robot_data[i][j]);
        j++;
        p = NULL;
      }

      if (j != 7)
      {
        printf("ロボットデータの項目数が一致しません. テキストファイルの内容に誤りが有ります.\n");
        exit(1);
      }

      //いくつ経路を読み込むのか判定
      //各ノードの経路番号の最大値を読み込み経路数として格納
      if (i == num_robot)
      {
        for (j = 1; j <= num_robot; j++)
        {
          if (buf_robot_data[j][2] > get_route_num)
          {
            get_route_num = buf_robot_data[j][2];
          }
        }
      }
    }
    else if (i <= (num_robot + get_route_num)) //ロボットの数+読み込む経路数
    {
      //経路情報(進行方向)格納
      route_num = i - num_robot;
      
      p = line;
      while (1)
      {
        buf = strtok(p, " ");
        
        if (buf == NULL)
          break;
        Route[route_num].m_Route[j] = atoi(buf);
        printf("Route[%d].m_Route[%d]=%d\n", route_num, j, Route[route_num].m_Route[j]);
        if (Route[route_num].m_Route[j] == STOP)
        {
          count_pause_point++; //一時停止位置のカウント
        }
        j++;
        p = NULL;
      }
      Route[route_num].m_Route_Len = j; //経路の区間数
      Route[route_num].m_Num_Pause_Point = count_pause_point; //経路中にある停止位置の数
      count_pause_point = 0;
    }
    else if (i <= (num_robot + 2 * get_route_num))
    {
      //経路の速度情報を格納
      route_num = i - (num_robot + get_route_num);
      p = line;
      while (1)
      {
        buf = strtok(p, " ");
        if (buf == NULL)
          break;
        Route[route_num].m_Section_Velocity[j] = atof(buf);
        printf("Route[%d].m_Section_Velocity[%d]=%f\n", route_num, j, Route[route_num].m_Section_Velocity[j]);
        j++;
        p = NULL;
      }
      //エラー処理
      if (Route[route_num].m_Route_Len != j)
      {
        printf("経路数と区間速度の数が一致しません.テキストファイルの内容に誤りが有ります.\n");
        exit(1);
      }
    }
    else if (i <= (num_robot + 3 * get_route_num))
    {
      //経路の停止時間情報格納
      route_num = i - (num_robot + 2 * get_route_num);
      p = line;
      while (Route[route_num].m_Num_Pause_Point != 0)
      {
        buf = strtok(p, " ");
        if (buf == NULL)
          break;
        Route[route_num].m_Pause_Time[j] = atof(buf);
        printf("Route[%d].m_Pause_Time[%d]=%f\n", route_num, j, Route[route_num].m_Pause_Time[j]);
        j++;
        p = NULL;
      }
      //エラー処理
      if (Route[route_num].m_Num_Pause_Point != j && Route[route_num].m_Num_Pause_Point != 0)
      {
        printf("停止位置の数と停止時間の数が一致しません.テキストファイルの内容に誤りがあります\n");
        exit(1);
      }
    }
    i++;
    j = 0;
  }

  fclose(fp);
}

void Set_Robot(int a_i_node, int exist) //二番目の引数に空回しでロボットを動かすなら1を、動かさないなら0を入れる
{

  int i;
  int num_route = buf_robot_data[a_i_node][2]; //何番目の経路の情報をセットするか
  int start_position = buf_robot_data[a_i_node][3]; //経路の何番目の情報から読み始めるか
  int route_len = Route[num_route].m_Route_Len; //経路の長さ

  Node[a_i_node].m_X = buf_robot_data[a_i_node][0];
  Node[a_i_node].m_Y = buf_robot_data[a_i_node][1];
  Node[a_i_node].m_Info = buf_robot_data[a_i_node][4];
  Node[a_i_node].m_Robot = TRUE;  //ノードがロボットかどうか
  Node[a_i_node].m_Exist = exist; //ノードが存在するかどうか
  NodeR[a_i_node].m_Num_Route = num_route;
  Node[a_i_node].m_Moved_num = start_position;
  NodeR[a_i_node].m_Transmission_distnace = sqrt(pow(buf_robot_data[a_i_node][6], 2.0) - pow(buf_robot_data[a_i_node][5], 2.0)); //飛行高度，通信可能距離から平面上で考えた時の通信可能距離を計算(= 99.874(m)) 関数powはべき乗 
  
  Robot_Next_Section(a_i_node, start_position, 0); //最初の経路番号に応じて速度を決める, ロボットの交差点を過ぎてからの経過時間を受け取り、超過分の処理を行う関数

  //各種フラグ初期化
  for (i = 0; i < ra_step_num; i++)
    Node[a_i_node].m_Avoid_Flag[i] = FALSE;

  Node[a_i_node].m_Restricted_Area_Num = -1;

  Node[a_i_node].m_Already_Set_Avoid_Flag = FALSE;

  Node[a_i_node].m_Received_From_FS = FALSE;
  for (i = 0; i < RA_STEP_NUM; i++)
    Node[a_i_node].m_Already_Count_intoRA[i] = FALSE;

  Node[a_i_node].m_Already_Count_ruinfo = FALSE;

  if (Sim_Flag == 1)
    Node[a_i_node].m_Enter_After_Idling = TRUE;
  else
    Node[a_i_node].m_Enter_After_Idling = FALSE;
}

//一度導入したロボットを削除するために使用
void Delete_Robot(int a_i_node, int exist)
{
  // Node[a_i_node].m_Exist = FALSE;

  // int i;
  // for (i = 0; i < ra_step_num; i++)
  //   Node[a_i_node].m_Avoid_Flag[i] = FALSE;

  // Node[a_i_node].m_Restricted_Area_Num = -1;

  // Node[a_i_node].m_Already_Set_Avoid_Flag = FALSE;

  // Node[a_i_node].m_Received_From_FS = FALSE;
  // for (i = 0; i < RA_STEP_NUM; i++)
  //   Node[a_i_node].m_Already_Count_intoRA[i] = FALSE;

  // Node[a_i_node].m_Already_Count_ruinfo = FALSE;

  // if (Sim_Flag == 1)
  //   Node[a_i_node].m_Enter_After_Idling = TRUE;
  // else
  //   Node[a_i_node].m_Enter_After_Idling = FALSE;

   int i;

  //ノードの存在を消す
  for (i = 1; i <= num_robot; i++)
  {
    Node[i].m_Exist = FALSE;
  }
}

