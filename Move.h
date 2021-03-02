//////////////////////////////////
// 移動と進路変更に関する関数群 //
//////////////////////////////////

////////////////
// 関数リスト //
//////////////////////////////////////////////////
// Set_Goal()
// Set_Goal2()
// Intersect_Line_RA()
// make_combination_code()
// Choose_next_route()
// Set_Route()
// Calc_Passable_Route()
// Is_Before_FS()
// Have_Passable_Route()
// Can_Make_Passable_Route()
// Move()
// Robot_Next_Section()
// Move_Robot()
// Set_Transmittable_Area()
// Set_Restricted_Area()
// set_restricted_area()
//////////////////////////////////////////////////

 int count_LU = 0;
 int count_RU = 0;
 int count_LT = 0;
 int count_RT = 0;

//生成された辺以外の3辺からGoalを選択
void Set_Goal(int a_i_node) //Node_Dist.hで使用 Set_Goal内でのm_X, m_Yはノードの発生地
{
  int ran;

  while (TRUE)
  {
    ran = rand03(); //0~3の整数をランダムに出力する

    if (ran == 0)
    {
      //W方向の辺へ
      Node[a_i_node].m_Goal_X = 0.0;
      Node[a_i_node].m_Goal_Y = A_i * (rand() % (block_num + 1)); //A_i(交差点の区間の長さ) * (0-12までの乱数)

      if ((Node[a_i_node].m_X < v * dt) || (Node[a_i_node].m_Y < v * dt && Node[a_i_node].m_Goal_Y < v * dt) || (Node[a_i_node].m_Y > Ay - v * dt && Node[a_i_node].m_Goal_Y > Ay - v * dt))
        continue; //ran = rand03() に戻る Goalの設定をし直し
      else
        break; //while文も抜ける Goalの設定完了
    }
    if (ran == 1)
    {
      //E方向の辺へ
      Node[a_i_node].m_Goal_X = Ax;
      Node[a_i_node].m_Goal_Y = A_i * (rand() % (block_num + 1));
      if ((Node[a_i_node].m_X > Ax - v * dt) || (Node[a_i_node].m_Y < v * dt && Node[a_i_node].m_Goal_Y < v * dt) || (Node[a_i_node].m_Y > Ay - v * dt && Node[a_i_node].m_Goal_Y > Ay - v * dt))
        continue;
      else
        break;
    }
    if (ran == 2)
    {
      //S方向の辺へ
      Node[a_i_node].m_Goal_Y = 0.0;
      Node[a_i_node].m_Goal_X = A_i * (rand() % (block_num + 1));
      if ((Node[a_i_node].m_Y < v * dt) || (Node[a_i_node].m_X < v * dt && Node[a_i_node].m_Goal_X < v * dt) || (Node[a_i_node].m_X > Ax - v * dt && Node[a_i_node].m_Goal_X > Ax - v * dt))
        continue;
      else
        break;
    }
    if (ran == 3)
    {
      //N方向の辺へ
      Node[a_i_node].m_Goal_Y = Ay;
      Node[a_i_node].m_Goal_X = A_i * (rand() % (block_num + 1));
      if ((Node[a_i_node].m_Y > Ay - v * dt) || (Node[a_i_node].m_X < v * dt && Node[a_i_node].m_Goal_X < v * dt) || (Node[a_i_node].m_X > Ax - v * dt && Node[a_i_node].m_Goal_X > Ax - v * dt))
        continue;
      else
        break;
    }
  }
}

void Set_Goal2(int a_i_node)
{
  int ran;
  int ran09;

  while (TRUE)
  {
    ran = rand03(); //0~3の整数をランダムに出力する
    //ran09 = rand09();
    
    if (ran == 0)
    {
      //W方向の辺へ
      Node[a_i_node].m_Goal_X = 0.0;
      Node[a_i_node].m_Goal_Y = A_i * (rand() % (block_num + 1)); //A_i(交差点の区間の長さ) * (0-12までの乱数)
      // if (Node[a_i_node].m_Goal_Y >= 1000 && Node[a_i_node].m_Goal_Y <= 5000)
      //   continue;
      
      if ((Node[a_i_node].m_X < v * dt) || (Node[a_i_node].m_Y < v * dt && Node[a_i_node].m_Goal_Y < v * dt) || (Node[a_i_node].m_Y > Ay - v * dt && Node[a_i_node].m_Goal_Y > Ay - v * dt))
        continue; //ran = rand03() に戻る Goalの設定をし直し
      else
        break; //while文も抜ける Goalの設定完了
    }
    if (ran == 1)
    {
      //E方向の辺へ
      Node[a_i_node].m_Goal_X = Ax;
      Node[a_i_node].m_Goal_Y = A_i * (rand() % (block_num + 1));
      // if (Node[a_i_node].m_Goal_Y >= 1000 && Node[a_i_node].m_Goal_Y <= 5000)
      //   continue;
      
      
      if ((Node[a_i_node].m_X > Ax - v * dt) || (Node[a_i_node].m_Y < v * dt && Node[a_i_node].m_Goal_Y < v * dt) || (Node[a_i_node].m_Y > Ay - v * dt && Node[a_i_node].m_Goal_Y > Ay - v * dt))
        continue;
      else
        break;
    }
    if (ran == 2)
    {
      //continue;
      //S方向の辺へ
      Node[a_i_node].m_Goal_Y = 0.0;
      Node[a_i_node].m_Goal_X = A_i * (rand() % (block_num + 1));
      // if (Node[a_i_node].m_Goal_X >= 1000 && Node[a_i_node].m_Goal_X <= 5000)
      //   continue;
      
      if ((Node[a_i_node].m_Y < v * dt) || (Node[a_i_node].m_X < v * dt && Node[a_i_node].m_Goal_X < v * dt) || (Node[a_i_node].m_X > Ax - v * dt && Node[a_i_node].m_Goal_X > Ax - v * dt))
        continue;
      else
        break;
    }
    if (ran == 3)
    {
      
      //N方向の辺へ
      Node[a_i_node].m_Goal_Y = Ay;
      Node[a_i_node].m_Goal_X = A_i * (rand() % (block_num + 1));

      // if (Node[a_i_node].m_Goal_X >= 1000 && Node[a_i_node].m_Goal_X <= 5000)
      //   continue;
      if ((Node[a_i_node].m_Y > Ay - v * dt) || (Node[a_i_node].m_X < v * dt && Node[a_i_node].m_Goal_X < v * dt) || (Node[a_i_node].m_X > Ax - v * dt && Node[a_i_node].m_Goal_X > Ax - v * dt))
        continue;
      else
        break;
    }
  }
}

//2点の座標を与えた時にそれらを結んだ直線が進入制限エリアRAと交わるかどうかを判定する関数
//交わるならTRUE，そうでないならFALSEを返す
int Intersect_Line_RA(int ra_no, double x, double y, double dest_x, double dest_y) //Moveで使用
{
  int i, j, k;
  double slope, temp_x, temp_y;

  slope = Slope(x, y, dest_x, dest_y); //Slope : 2点間を結ぶ直線の傾きを求める関数
  temp_x = 0.0;
  temp_y = 0.0;
  i = 0;
  while (TRUE)
  {

    //傾きがあるとき
    if (!d_Equal(slope, 0.0))
    {
      //目的地のある方向に進んだ時を考える
      if (dest_x > x)
        temp_x = x + A_i / 100.0 * i; //x + ( (A_i * i) / 100 )
      else if (dest_x < x)
        temp_x = x - A_i / 100.0 * i;
      temp_y = y + slope * (temp_x - x);
    }

    //直線がy軸と平行なとき
    else if (d_Equal(slope, 0.0) && d_Equal(x, dest_x))
    {
      if (dest_y > y)
        temp_y = y + A_i / 100.0 * i;
      else if (dest_y < y)
        temp_y = y - A_i / 100.0 * i;
      temp_x = x;
    }

    //直線がx軸と平行なとき
    else if (d_Equal(slope, 0.0) && d_Equal(y, dest_y))
    {
      if (dest_x > x)
        temp_x = x + A_i / 100.0 * i;
      else if (dest_x < x)
        temp_x = x - A_i / 100.0 * i;
      temp_y = y;
    }

    //制限エリアを通るか判定
    for (j = 0; j < ra_num[ra_no]; j++) 
    {
      if (RA[ra_no][j].m_X1 <= temp_x && temp_x <= RA[ra_no][j].m_X2 && RA[ra_no][j].m_Y1 <= temp_y && temp_y <= RA[ra_no][j].m_Y2)
      {
        return TRUE; //RAと交わる
      }
    }

    //サービスエリア内だけを判定する
    if (temp_x < 0.0 || temp_x > Ax || temp_y < 0.0 || temp_y > Ay)
      break;
    i++;
  }
  //RAと交わらない
  return FALSE;
}

//格子状道路の最短経路の組み合わせを計算
//計算結果は符号（縦を0，横を1とした長さnのビット列において何番目に1があるかを示す）
//n:総ステップ数，t:横方向に進む回数，route_c:組み合わせ符号を格納する2次元配列
//戻り値は総経路数
//Donald E. Knuth,
//``The Art of Computer Programming Volume 4, Fascicle 3 Generating All Combinations and Partitions 日本語版,''
//アスキー, 東京, 2008, pp. 5, アルゴリズム T
int make_combination_code(int n, int t, int **route_c) //Moveで使用
{
  int i;
  int j, x, c[t + 3];
  int route_num = 0; //総経路数

  for (j = 1; j <= t; j++)
  {
    c[j] = j - 1;
  }
  c[t + 1] = n;
  c[t + 2] = 0;
  j = t;

  while (TRUE)
  {

    for (i = 1; i <= t; i++)
    {
      route_c[route_num][i - 1] = c[i];
    }
    route_num++;

    if (j > 0)
    {
      x = j;
      //T6
      c[j] = x;
      j--;
      continue;
    }

    //T3
    if ((c[1] + 1) < c[2])
    {
      c[1]++;
      continue;
    }
    else
      j = 2;

    //T4
    while (TRUE)
    {
      c[j - 1] = j - 2;
      x = c[j] + 1;
      if (x != c[j + 1])
        break;
      else
        j++;
    }

    //T5
    if (j > t)
      break;

    //T6
    c[j] = x;
    j--;
  }
  return route_num;
}

//次の進路を選んで方向を示すマクロ定数を返す関数
int Choose_next_route(int a_i_node, int remaining_num_x, int remaining_num_y) //Move.h, 関数Set_Route内で使用
{
  double node_X = Node[a_i_node].m_X;
  double node_Y = Node[a_i_node].m_Y;
  double node_Goal_X = Node[a_i_node].m_Goal_X;
  double node_Goal_Y = Node[a_i_node].m_Goal_Y;
  double node_Vx = Node[a_i_node].m_Vx; //ノードの速度
  double node_Vy = Node[a_i_node].m_Vy; //ノードの速度

  double route_num_x, route_num_y;
  double p_x;
  double ran;

  //x方向，y方向それぞれについて残りの経路数を調べる
  if (remaining_num_x > 0) //remaining_num_x : 残りのx方向の距離(経路数)
    route_num_x = (double)fact(remaining_num_x - 1 + remaining_num_y) / (fact(remaining_num_x - 1) * fact(remaining_num_y)); //関数factは階乗の計算をする関数(Common.hで定義) 現在地から1個分x方向に進むと確定した場合の残りの考えられる経路数
  else
    route_num_x = 0.0;

  if (remaining_num_y > 0)
    route_num_y = (double)fact(remaining_num_y - 1 + remaining_num_x) / (fact(remaining_num_y - 1) * fact(remaining_num_x));
  else
    route_num_y = 0.0;

  //x, yの両方向の経路がないなら目的地
  if (d_Equal(route_num_x, 0.0) && d_Equal(route_num_y, 0.0))
    return EXIT;

  //x方向に進む確率
  p_x = route_num_x / (route_num_x + route_num_y);

  ran = rand01();
  //x方向に進むとき
  if (ran < p_x)
  {
    //目的地の方向に応じた方向を返す
    if (node_Goal_X < node_X)
      return LEFT;
    if (node_Goal_X > node_X)
      return RIGHT;
  }
  //y方向に進むとき
  else
  {
    if (node_Goal_Y < node_Y)
      return DOWN;
    if (node_Goal_Y > node_Y)
      return UP;
  }
}

//サービスエリアを出るまでの経路を決定して各ノードを表す構造体へ保存する
void Set_Route(int a_i_node) //Node_Dist.h, 関数Dist_in_Interval内で使用
{
  int i, route;
  int remaining_num_x, remaining_num_y;

  //x, yの各方向に進む回数を計算
  if (Node[a_i_node].m_X < 0.0) //ノードのx座標が負(SAの左外)
    remaining_num_x = (int)(fabs(Node[a_i_node].m_Goal_X - 0.0) / A_i + 0.5); //fabs()は絶対値を返す関数, A_i:交差点の区間の長さ
  else if (Node[a_i_node].m_X > Ax) //ノードのx座標がAxより大きい(SAの右外)
    remaining_num_x = (int)(fabs(Node[a_i_node].m_Goal_X - Ax) / A_i + 0.5); //0.5は補正のため
  else
    remaining_num_x = (int)(fabs(Node[a_i_node].m_Goal_X - Node[a_i_node].m_X) / A_i + 0.5);

  if (Node[a_i_node].m_Y < 0.0) //ノードのy座標が負(SAの下外)
    remaining_num_y = (int)(fabs(Node[a_i_node].m_Goal_Y - 0.0) / A_i + 0.5);
  else if (Node[a_i_node].m_Y > Ay) //ノードのx座標がAyより大きい(SAの上外)
    remaining_num_y = (int)(fabs(Node[a_i_node].m_Goal_Y - Ay) / A_i + 0.5);
  else
    remaining_num_y = (int)(fabs(Node[a_i_node].m_Goal_Y - Node[a_i_node].m_Y) / A_i + 0.5);

  //1ブロック分ずつ進路を決定して格納
  i = 0;
  do
  {
    route = Choose_next_route(a_i_node, remaining_num_x, remaining_num_y);

    if (route == LEFT || route == RIGHT)
      remaining_num_x--;
    if (route == UP || route == DOWN)
      remaining_num_y--;

    Node[a_i_node].m_Route[i] = route; //経路を格納
    i++;
  } while (route != EXIT);
  //はじめは0番目の経路を読む
  Node[a_i_node].m_Moved_num = 0;
  
}

//Goalまでのすべての経路を調べ，RAを通過しない経路を1つ取得する
void Calc_Passable_Route(int a_i_node, double X, double Y, double Goal_X, double Goal_Y, int moved_num, int ra_no)
{

  int i, j, route_i, ra_i;
  int step_num, remaining_num_x, remaining_num_y;
  int horizontal, vertical;
  int route_num; //総経路数
  int route_max_num = 2000000;
  int **route_c; //route_c:組み合わせ符号を格納する2次元配列
  ra_no = 0;
  
  //各方向に進む交差点数を計算
  remaining_num_x = (int)(fabs(Goal_X - X) / A_i + 0.5); //x方向の交差点数
  remaining_num_y = (int)(fabs(Goal_Y - Y) / A_i + 0.5); //y方向の交差点数
  step_num = remaining_num_x + remaining_num_y; //合計の交差点数

  //メモリの動的確保と初期化
  if ((route_c = (int **)malloc(sizeof(int *) * route_max_num)) == NULL)
  {
    puts("memory aallocation error ( route_c )");
    exit(EXIT_FAILURE);
  }
  for (route_i = 0; route_i < route_max_num; route_i++)
  {
    if ((route_c[route_i] = (int *)malloc(sizeof(int) * (remaining_num_x + 1))) == NULL)
    {
      puts("memory aallocation error ( route_c[] )");
      exit(EXIT_FAILURE);
    }
  }
  for (i = 0; i < route_max_num; i++)
  {
    for (j = 0; j < remaining_num_x + 1; j++)
    {
      route_c[i][j] = -1;
    }
  }

  //x方向にしか進めない場合(remaining_num_y = 0)
  if (remaining_num_x == step_num)
  {
    for (j = 0; j < step_num; j++)
    {
      route_c[0][j] = j;
    }
    route_num = 1;
  }
  //そうでない場合 (x方向，y方向ともに進める)
  else if (remaining_num_x > 0)
  {
    //経路を表す符号の生成
    route_num = make_combination_code(step_num, remaining_num_x, route_c);
  }
  else //y方向にしか進めない場合
  {
    route_num = 1;
  }

  int **route, *route_list;
  int passable_flag, passable_route_num = 0; //通行できる経路数
  double temp_x, temp_y;

  //メモリの動的確保
  if ((route = (int **)malloc(sizeof(int *) * route_num)) == NULL)
  {
    puts("memory aallocation error ( route )");
    exit(EXIT_FAILURE);
  }
  for (route_i = 0; route_i < route_num; route_i++)
  {
    if ((route[route_i] = (int *)malloc(sizeof(int) * step_num)) == NULL)
    {
      puts("memory aallocation error ( route[] )");
      exit(EXIT_FAILURE);
    }
  }
  //メモリの動的確保と初期化
  if ((route_list = (int *)malloc(sizeof(int) * (route_num))) == NULL)
  {
    puts("memory aallocation error ( route_list )");
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < route_num; i++)
  {
    route_list[i] = -1; //初期化
  }

  //目的地と現在位置との差から，進む方向を求める．(X, Yは現在の進行方向を判定している交差点のx座標, y座標)
  if (Goal_X - X > 0)
    horizontal = RIGHT;
  else
    horizontal = LEFT;

  if (Goal_Y - Y > 0)
    vertical = UP;
  else
    vertical = DOWN;

  //符号化された経路route_cをデコード
  //同時にRAを通過するかどうか判定 -> 通過することが確認できた時点でその経路は無視する
  for (route_i = 0; route_i < route_num; route_i++)
  {
    j = 0;
    passable_flag = TRUE;
    temp_x = X;
    temp_y = Y;
    for (i = 0; i < step_num; i++)
    {
      if (i == route_c[route_i][j] && j < remaining_num_x)
      {
        route[route_i][i] = horizontal; //目的地と現在位置の差より，RIGHTまたはLEFTを格納
        j++;
        if (horizontal == RIGHT)
        {
          temp_x += A_i; //交差点1つ分足す
        }
        if (horizontal == LEFT)
        {
          temp_x -= A_i; //交差点1つ分引く
        }
      }
      else
      {
        route[route_i][i] = vertical;
        if (vertical == UP)
        {
          temp_y += A_i; //交差点1つ分足す
        }
        if (vertical == DOWN)
        {
          temp_y -= A_i; //交差点1つ分引く
        }
      }
      //進入制限エリア内なら通れない
      for (ra_i = 0; ra_i < ra_num[ra_no]; ra_i++)
      {
        if (RA[ra_no][ra_i].m_X1 <= temp_x && temp_x <= RA[ra_no][ra_i].m_X2 && RA[ra_no][ra_i].m_Y1 <= temp_y && temp_y <= RA[ra_no][ra_i].m_Y2)
        {
          passable_flag = FALSE;
          break;
        }

      }
      //通れないなら次の経路を調べる
      if (passable_flag == FALSE)
        break;
    }
    //通れる経路ならその要素番号と通れる経路数を+1
    if (passable_flag == TRUE)
    {
      route_list[passable_route_num] = route_i; //配列route_listにroute_i番の経路番号を格納
      passable_route_num++; //通行できる経路数を+1
    }
  }

  //通過できる経路が1つでもある
  if (passable_route_num > 0)
  {
    //ランダムに一つ選ぶ
    int ran = rand();
    int choosed_route = route_list[ran % passable_route_num];
    //ノードの経路を更新
    for (i = 0; i < step_num; i++)
    {
      Node[a_i_node].m_Route[moved_num + i] = route[choosed_route][i]; //ノードの経路を選ばれた経路に設定
    }
    //サービスエリアから出る経路であれば最後にEXITを追加
    if (d_Equal(Goal_X, Node[a_i_node].m_Goal_X) && d_Equal(Goal_Y, Node[a_i_node].m_Goal_Y))
      Node[a_i_node].m_Route[moved_num + i] = EXIT;
  }

  //必要以上にメモリを確保しておかないようにここでメモリ解放する
  for (i = 0; i < route_max_num; i++)
  {
    free(route_c[i]);
  }
  for (i = 0; i < route_num; i++)
  {
    free(route[i]);
  }
  free(route_c);
  free(route);
  free(route_list);

  int candidate_point_num = 0; //候補点の数
  double Dest_X, Dest_Y;
  double angle, temp_angle;
  double candidate_x[100], candidate_y[100];

  double FS_X, FS_Y;
  FS_X = Node[0].m_X; //FSのx座標
  FS_Y = Node[0].m_Y; //FSのy座標

  //通過できる経路が1つもないとき
  //遠回りするときの通過点を求める
  if (passable_route_num == 0) //通行できる経路の数が0
  {
    angle = 2 * M_PI;
    Dest_X = 0.0;
    Dest_Y = 0.0;
    //RAを含む最小の長方形（正方形）領域の点を全て調べる
    for (i = 0; i <= block_num; i++)
    {
      for (j = 0; j <= block_num; j++)
      {
        if (d_Equal(X, A_i * i) && d_Equal(Y, A_i * j))
          continue;
        if (FS_X - (d_ra[ra_no] + 0.5) * A_i > A_i * i || A_i * i > FS_X + (d_ra[ra_no] + 0.5) * A_i || FS_Y - (d_ra[ra_no] + 0.5) * A_i > A_i * j || A_i * j > FS_Y + (d_ra[ra_no] + 0.5) * A_i) //RAを含む最小の長方形の外側ならcontinue
          continue;
        //その点と今いる点を結んだ直線が禁止エリアと交わらないなら通過点の候補になる (A_i * i, A_i * j)はRAを含む最小の長方形（正方形）領域の点
        if (Intersect_Line_RA(ra_no, X, Y, A_i * i, A_i * j) == TRUE) //2点の座標を与えた時にそれらを結んだ直線が進入制限エリアRAと交わるかどうかを判定する関数 交わる(TRUE)ならcontinue
          continue;
        //現在地と目的地，現在地と中継点それぞれを通るベクトル間の角度を求めてそれが最も小さいものを選ぶ
        //同じ角度の時は距離がなるべく遠いものを選ぶ
        temp_angle = Angle_Of_2Vector(X, Y, Goal_X, Goal_Y, A_i * i, A_i * j); //現在地と目的地，現在地と中継点それぞれを通るベクトル間の角度を計算
        if (angle > temp_angle)
        {
          angle = temp_angle;
          Dest_X = A_i * i;
          Dest_Y = A_i * j;
          candidate_point_num = 0;
        }
        else if (d_Equal(angle, temp_angle)) //なす角が0(2 * M_PI)
        {
          angle = temp_angle;

          if (d_Equal(sqrt2((Dest_X - X), (Dest_Y - Y)), sqrt2((A_i * i - X), (A_i * j - Y)))) 
          {
            if (candidate_point_num == 0) //候補点の数が0なら
            {
              candidate_x[0] = Dest_X;
              candidate_y[0] = Dest_Y;
              candidate_x[1] = A_i * i;
              candidate_y[1] = A_i * j;
              candidate_point_num = 2;
            }
            else //候補点の数が0でなければ
            {
              candidate_x[candidate_point_num] = A_i * i;
              candidate_y[candidate_point_num] = A_i * j;
              candidate_point_num++;
            }
          }
          else if (sqrt2((Dest_X - X), (Dest_Y - Y)) < sqrt2((A_i * i - X), (A_i * j - Y))) //目的地と現在地の距離 < 中継点と現在地の距離
          {
            Dest_X = A_i * i;
            Dest_Y = A_i * j;
            candidate_point_num = 0;
          }
        }
      }
    }

    if (candidate_point_num > 0)
    {
      int ran = rand();
      Dest_X = candidate_x[ran % candidate_point_num];
      Dest_Y = candidate_y[ran % candidate_point_num];
    }

    //現在位置から通過点，通過点からサービスエリアを出る時の点それぞれについて経路を検索する
    Calc_Passable_Route(a_i_node, X, Y, Dest_X, Dest_Y, moved_num, ra_no); //現在位置から通過点

    int rem_x = (int)(fabs(Dest_X - X) / A_i + 0.5);
    int rem_y = (int)(fabs(Dest_Y - Y) / A_i + 0.5);
    Calc_Passable_Route(a_i_node, Dest_X, Dest_Y, Goal_X, Goal_Y, moved_num + rem_x + rem_y, ra_no); //通過点からサービスエリアを出る時の点
  }
}

//目的地方向にFSがあるかどうか
int Is_Before_FS(int a_i_node)
{
  double FS_X, FS_Y;
  FS_X = Node[0].m_X;
  FS_Y = Node[0].m_Y;

  if (Node[a_i_node].m_Goal_X > Node[a_i_node].m_X && Node[a_i_node].m_X <= FS_X + epsilon) //epsilon = 0.0001
    return TRUE;
  if (Node[a_i_node].m_Goal_X < Node[a_i_node].m_X && Node[a_i_node].m_X >= FS_X - epsilon)
    return TRUE;
  if (Node[a_i_node].m_Goal_Y > Node[a_i_node].m_Y && Node[a_i_node].m_Y <= FS_Y + epsilon)
    return TRUE;
  if (Node[a_i_node].m_Goal_Y < Node[a_i_node].m_Y && Node[a_i_node].m_Y >= FS_Y - epsilon)
    return TRUE;

  return FALSE;
}

//今持っている経路が禁止エリアを通るかどうか判定する
int Have_Passable_Route(int a_i_node, int ra_no)
{
  int i;
  int next_route;
  int moved_num;
  double temp_x, temp_y;
  ra_no = 0;
  
  temp_x = Node[a_i_node].m_X;
  temp_y = Node[a_i_node].m_Y;
  moved_num = Node[a_i_node].m_Moved_num; //どこまで進んだかを示す（次にm_Route[]の何番目の要素を読むかを指定）

  
  do
  {
    next_route = Node[a_i_node].m_Route[moved_num]; //m_Routeは経路を格納

    if (next_route == LEFT)
    {
      temp_x -= A_i;
      moved_num++;
    }
    if (next_route == RIGHT)
    {
      temp_x += A_i;
      moved_num++;
    }
    if (next_route == UP)
    {
      temp_y += A_i;
      moved_num++;
    }
    if (next_route == DOWN)
    {
      temp_y -= A_i;
      moved_num++;
    }
    for (i = 0; i < ra_num[ra_no]; i++) //ra_num : 進入制限エリアを表す構造体の数
    {
      if (RA[ra_no][i].m_X1 <= temp_x && temp_x <= RA[ra_no][i].m_X2 && RA[ra_no][i].m_Y1 <= temp_y && temp_y <= RA[ra_no][i].m_Y2)
      {
        return FALSE; //通る
      }
    }

  } while (Node[a_i_node].m_Route[moved_num] != EXIT); //Calc_Passable_Route内で，m_Routeにサービスエリアから出る経路であれば最後にEXITを追加

  return TRUE; //通らない
}

int Can_Make_Passable_Route(int a_i_node, int ra_no) //RA回避ルートを作ることができるかどうか
{
  int i;
  double alpha = A_i / 20.0; //25

  for (i = 0; i < ra_num[ra_no]; i++)
  {
    if ((RA[ra_no][i].m_X1 + alpha) <= Node[a_i_node].m_X && Node[a_i_node].m_X <= (RA[ra_no][i].m_X2 - alpha) && (RA[ra_no][i].m_Y1 - alpha) <= Node[a_i_node].m_Y && Node[a_i_node].m_Y <= (RA[ra_no][i].m_Y2 + alpha) && ra_num[ra_no] / 2 < i)
    {
      return FALSE; //ノードはRAのある道路上に存在する(RA回避ルートを作ることができない)
    }
    if ((RA[ra_no][i].m_X1 - alpha) <= Node[a_i_node].m_X && Node[a_i_node].m_X <= (RA[ra_no][i].m_X2 + alpha) && (RA[ra_no][i].m_Y1 + alpha) <= Node[a_i_node].m_Y && Node[a_i_node].m_Y <= (RA[ra_no][i].m_Y2 - alpha) && ra_num[ra_no] / 2 >= i)
    {
      return FALSE;
    }
  }
  return TRUE; //ノードはRAのある道路上に存在しない(RA回避ルートを作ることができる)
}

//経路を格納した配列を読んでそれに応じてノードを移動させる関数
void Move(int a_i_node)
{
  double node_X, node_Y, node_Vx, node_Vy, node_dx, node_dy, node_Goal_X, node_Goal_Y;
  int node_Moved_num, node_Route, node_ra_num;
  double FS_X, FS_Y;
  int i, j, i_ra_no;
  double ran;

  node_X = Node[a_i_node].m_X;
  node_Y = Node[a_i_node].m_Y;
  node_Vx = Node[a_i_node].m_Vx;
  node_Vy = Node[a_i_node].m_Vy;
  node_dx = Node[a_i_node].m_Vx * dt; //1ステップでx方向に進む距離 dt = 0.1
  node_dy = Node[a_i_node].m_Vy * dt;
  node_Moved_num = Node[a_i_node].m_Moved_num;
  node_Goal_X = Node[a_i_node].m_Goal_X;
  node_Goal_Y = Node[a_i_node].m_Goal_Y;
  node_ra_num = Node[a_i_node].m_Restricted_Area_Num; //何番目のRAを迂回しているか
  FS_X = Node[0].m_X;
  FS_Y = Node[0].m_Y;
  //W->E
  if (node_Vx > 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      //交差点を超える
      if (node_X <= A_i * i && node_X + node_dx > A_i * i)
      {
        //とりあえず交差点まで移動しておく
        Node[a_i_node].m_X = A_i * i;

        //情報が必要であったかどうかを調べる
        if (Node[a_i_node].m_Info == TRUE && Node[a_i_node].m_Already_Count_ruinfo == FALSE) //m_Already_Count_ruinfoはすでに不必要な情報を受け取ったかどうかについてカウントしたかどうか
        {
          for (i_ra_no = ra_step_num; i_ra_no < ra_step_num + count_area_num; i_ra_no++) //ra_step_num = 1
          {
            if (Have_Passable_Route(a_i_node, i_ra_no) == TRUE) //今持っている経路が禁止エリアを通るかどうか判定する, TRUEはRAを通らない
            {
              num_recieved_unnecessary_info[i_ra_no]++; //今持っている経路がRAを通らなかったら，不必要な情報をもらったとしてカウント
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_unnecessary_info_from_robot[i_ra_no]++; //その情報をロボットからもらっていたら，不必要な情報をロボットからもらったとしてカウント
              }
            }
            else //Have_Passable_Route() = FALSE, FALSEはRAを通る
            {
              num_recieved_necessary_info[i_ra_no]++; //今持っている経路がRAを通るなら，必要な情報をもらったとしてカウント
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_necessary_info_from_robot[i_ra_no]++; //その情報をロボットからもらっていたら，必要な情報をロボットからもらったとしてカウント
              }
            }
          }
          Node[a_i_node].m_Already_Count_ruinfo = TRUE;
        }
        //行動変化するかを判定する
        //RA外であり,情報を持っていて，進行方向にFSがあり，避けるかどうかまだ判定していないもの
        if (Node[a_i_node].m_Info == TRUE && Is_Before_FS(a_i_node) == TRUE && node_ra_num < 0 && Node[a_i_node].m_Already_Set_Avoid_Flag == FALSE && Can_Make_Passable_Route(a_i_node, 0) == TRUE) //Can_Make_Passable_Route() = TRUE はこれからRA回避ルートを作ることができる
        {

          //RAを併用するかどうか
          //2つ併用する場合と併用しない場合の2パターンに対応
          if (ra_step_num > 1)
          {
            ran = rand01();
            if (ran < q) //q : RA1を提示する確率（RAを併用するときだけ使う）
              Node[a_i_node].m_Restricted_Area_Num = 0;
            else
              Node[a_i_node].m_Restricted_Area_Num = 1;
          }
          if (ra_step_num == 1)
            Node[a_i_node].m_Restricted_Area_Num = 0;

          node_ra_num = Node[a_i_node].m_Restricted_Area_Num;

          //元々の経路では禁止エリアに入るもの
          if (Have_Passable_Route(a_i_node, node_ra_num) == FALSE) //RAを通る
          {
            ran = rand01();
            //確率p_avoで避ける p_avo : 情報を受け取ったノードが事故現場を避ける確率
            if (ran < p_avo[node_ra_num])
            {
              Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE; //既に迂回動作をしているかを示す
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = AVOID; //避ける m_Avoid_Flagは行動変化を示すフラグ
              //経路を計算する
              Calc_Passable_Route(a_i_node, A_i * i, node_Y, node_Goal_X, node_Goal_Y, node_Moved_num, node_ra_num); //A_i * iは今見てる交差点
            }
            else
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = THROUGH; //避けない
          }
          Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
        }

        //これから進む方向を取得
        node_Route = Node[a_i_node].m_Route[node_Moved_num]; //Calc_Passable_Route内で，RIGHT, LEFT, ... を格納

        if (node_Route == EXIT) //SAからでるノード
        {
          Node[a_i_node].m_Exist = FALSE; //ノードの存在を消す
          return;
        }

        if (node_Route == RIGHT)
        {
          Node[a_i_node].m_X = node_X + node_dx; //交差点を超える場合に，とりあえず交差点の移動させておいてるから，m_Xをもともとの交差点を超えたところの座標にする
        }
        if (node_Route == LEFT)
        {
          Node[a_i_node].m_X -= (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vx = -node_Vx;
        }
        if (node_Route == UP)
        {
          Node[a_i_node].m_Y += (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vy = node_Vx;
          Node[a_i_node].m_Vx = 0.0;
        }
        if (node_Route == DOWN)
        {
          Node[a_i_node].m_Y -= (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vy = -node_Vx;
          Node[a_i_node].m_Vx = 0.0;
        }
        //端末数を調べる
        for (i_ra_no = 0; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
        {
          if (Node[a_i_node].m_Already_Count_intoRA[0] == FALSE) //すでにRAに進入したノードとしてカウントしたかどうか
          {
            Count_Having_Info_Nodes_Num_Into_RA(0, a_i_node, node_X, node_Y, node_Route); //Commonで定義 空回し終了後から情報を持って禁止エリアに入る端末と持たずに禁止エリアに入る端末の数を調べる
          }
        }
        //読んだ経路の数を+1
        Node[a_i_node].m_Moved_num++;
        break;
      }
    }
    //交差点を超えていない時
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
    }
    return;
  }
  //E->W
  if (node_Vx < 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_X >= A_i * i && node_X + node_dx < A_i * i)
      {
        Node[a_i_node].m_X = A_i * i;

        if (Node[a_i_node].m_Info == TRUE && Node[a_i_node].m_Already_Count_ruinfo == FALSE)
        {
          for (i_ra_no = ra_step_num; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
          {
            if (Have_Passable_Route(a_i_node, i_ra_no) == TRUE) //今持っている経路が禁止エリアを通るかどうか判定する, TRUEはRAを通らない
            {
              
              num_recieved_unnecessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_unnecessary_info_from_robot[i_ra_no]++;
              }
            }
            else //RAを通る
            {
              num_recieved_necessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_necessary_info_from_robot[i_ra_no]++;
              }
            }
          }
          Node[a_i_node].m_Already_Count_ruinfo = TRUE;
        }

        if (Node[a_i_node].m_Info == TRUE && Is_Before_FS(a_i_node) == TRUE && node_ra_num < 0 && Node[a_i_node].m_Already_Set_Avoid_Flag == FALSE && Can_Make_Passable_Route(a_i_node, 0) == TRUE)
        {

          if (ra_step_num > 1)
          {
            ran = rand01();
            if (ran < q)
              Node[a_i_node].m_Restricted_Area_Num = 0;
            else
              Node[a_i_node].m_Restricted_Area_Num = 1;
          }
          if (ra_step_num == 1)
            Node[a_i_node].m_Restricted_Area_Num = 0;

          node_ra_num = Node[a_i_node].m_Restricted_Area_Num; //m_Restricted_Area_Numは何番目のRAを迂回しているか

          if (Have_Passable_Route(a_i_node, node_ra_num) == FALSE)
          {
            ran = rand01();
            if (ran < p_avo[node_ra_num])
            {
              Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = AVOID;

              Calc_Passable_Route(a_i_node, A_i * i, node_Y, node_Goal_X, node_Goal_Y, node_Moved_num, node_ra_num);
            }
            else
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = THROUGH;
          }
          Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
        }

        node_Route = Node[a_i_node].m_Route[node_Moved_num];

        if (node_Route == EXIT)
        {
          Node[a_i_node].m_Exist = FALSE;
          return;
        }

        if (node_Route == RIGHT)
        {
          Node[a_i_node].m_X -= (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vx = -node_Vx;
        }
        if (node_Route == LEFT)
        {
          Node[a_i_node].m_X = node_X + node_dx;
        }
        if (node_Route == UP)
        {
          Node[a_i_node].m_Y -= (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vy = -node_Vx;
          Node[a_i_node].m_Vx = 0.0;
        }
        if (node_Route == DOWN)
        {
          Node[a_i_node].m_Y += (node_X + node_dx - A_i * i);
          Node[a_i_node].m_Vy = node_Vx;
          Node[a_i_node].m_Vx = 0.0;
        }
        for (i_ra_no = 0; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
        {
          if (Node[a_i_node].m_Already_Count_intoRA[0] == FALSE)
          {
            Count_Having_Info_Nodes_Num_Into_RA(0, a_i_node, node_X, node_Y, node_Route);
          }
        }
        Node[a_i_node].m_Moved_num++;
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
    }
    return;
  }
  //S->N
  if (node_Vy > 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_Y <= A_i * i && node_Y + node_dy > A_i * i)
      {
        Node[a_i_node].m_Y = A_i * i;

        if (Node[a_i_node].m_Info == TRUE && Node[a_i_node].m_Already_Count_ruinfo == FALSE)
        {
          for (i_ra_no = ra_step_num; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
          {
            if (Have_Passable_Route(a_i_node, i_ra_no) == TRUE)
            {
              num_recieved_unnecessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_unnecessary_info_from_robot[i_ra_no]++;
              }
            }
            else
            {
              num_recieved_necessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_necessary_info_from_robot[i_ra_no]++;
              }
            }
          }
          Node[a_i_node].m_Already_Count_ruinfo = TRUE;
        }

        if (Node[a_i_node].m_Info == TRUE && Is_Before_FS(a_i_node) == TRUE && node_ra_num < 0 && Node[a_i_node].m_Already_Set_Avoid_Flag == FALSE && Can_Make_Passable_Route(a_i_node, 0) == TRUE)
        {

          if (ra_step_num > 1)
          {
            ran = rand01();
            if (ran < q)
              Node[a_i_node].m_Restricted_Area_Num = 0;
            else
              Node[a_i_node].m_Restricted_Area_Num = 1;
          }
          if (ra_step_num == 1)
            Node[a_i_node].m_Restricted_Area_Num = 0;

          node_ra_num = Node[a_i_node].m_Restricted_Area_Num;

          if (Have_Passable_Route(a_i_node, node_ra_num) == FALSE)
          {
            ran = rand01();
            if (ran < p_avo[node_ra_num])
            {
              Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = AVOID;

              Calc_Passable_Route(a_i_node, node_X, A_i * i, node_Goal_X, node_Goal_Y, node_Moved_num, node_ra_num);
            }
            else
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = THROUGH;
          }
          Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
        }

        node_Route = Node[a_i_node].m_Route[node_Moved_num];

        if (node_Route == EXIT)
        {
          Node[a_i_node].m_Exist = FALSE;
          return;
        }

        if (node_Route == RIGHT)
        {
          Node[a_i_node].m_X += (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vx = node_Vy;
          Node[a_i_node].m_Vy = 0.0;
        }
        if (node_Route == LEFT)
        {
          Node[a_i_node].m_X -= (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vx = -node_Vy;
          Node[a_i_node].m_Vy = 0.0;
        }
        if (node_Route == UP)
        {
          Node[a_i_node].m_Y = node_Y + node_dy;
        }
        if (node_Route == DOWN)
        {
          Node[a_i_node].m_Y -= (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vy = -node_Vy;
        }

        for (i_ra_no = 0; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
        {
          if (Node[a_i_node].m_Already_Count_intoRA[0] == FALSE)
          {
            Count_Having_Info_Nodes_Num_Into_RA(0, a_i_node, node_X, node_Y, node_Route);
          }
        }
        Node[a_i_node].m_Moved_num++;
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
    }
    return;
  }

  //N->S
  if (node_Vy < 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_Y >= A_i * i && node_Y + node_dy < A_i * i)
      {
        Node[a_i_node].m_Y = A_i * i;

        if (Node[a_i_node].m_Info == TRUE && Node[a_i_node].m_Already_Count_ruinfo == FALSE)
        {
          for (i_ra_no = ra_step_num; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
          {
            if (Have_Passable_Route(a_i_node, i_ra_no) == TRUE)
            {
              num_recieved_unnecessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_unnecessary_info_from_robot[i_ra_no]++;
              }
            }
            else
            {
              num_recieved_necessary_info[i_ra_no]++;
              if (Node[a_i_node].m_Received_From_Robot == TRUE)
              {
                num_recieved_necessary_info_from_robot[i_ra_no]++;
              }
            }
          }
          Node[a_i_node].m_Already_Count_ruinfo = TRUE;
        }

        if (Node[a_i_node].m_Info == TRUE && Is_Before_FS(a_i_node) == TRUE && node_ra_num < 0 && Node[a_i_node].m_Already_Set_Avoid_Flag == FALSE && Can_Make_Passable_Route(a_i_node, 0) == TRUE)
        {

          if (ra_step_num > 1)
          {
            ran = rand01();
            if (ran < q)
              Node[a_i_node].m_Restricted_Area_Num = 0;
            else
              Node[a_i_node].m_Restricted_Area_Num = 1;
          }
          if (ra_step_num == 1)
            Node[a_i_node].m_Restricted_Area_Num = 0;

          node_ra_num = Node[a_i_node].m_Restricted_Area_Num;

          if (Have_Passable_Route(a_i_node, node_ra_num) == FALSE)
          {
            ran = rand01();
            if (ran < p_avo[node_ra_num])
            {
              Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = AVOID;

              Calc_Passable_Route(a_i_node, node_X, A_i * i, node_Goal_X, node_Goal_Y, node_Moved_num, node_ra_num);
            }
            else
              Node[a_i_node].m_Avoid_Flag[node_ra_num] = THROUGH;
          }
          Node[a_i_node].m_Already_Set_Avoid_Flag = TRUE;
        }

        node_Route = Node[a_i_node].m_Route[node_Moved_num];

        if (node_Route == EXIT)
        {
          Node[a_i_node].m_Exist = FALSE;
          return;
        }

        if (node_Route == RIGHT)
        {
          Node[a_i_node].m_X -= (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vx = -node_Vy;
          Node[a_i_node].m_Vy = 0.0;
        }
        if (node_Route == LEFT)
        {
          Node[a_i_node].m_X += (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vx = node_Vy;
          Node[a_i_node].m_Vy = 0.0;
        }
        if (node_Route == UP)
        {
          Node[a_i_node].m_Y -= (node_Y + node_dy - A_i * i);
          Node[a_i_node].m_Vy = -node_Vy;
        }
        if (node_Route == DOWN)
        {
          Node[a_i_node].m_Y = node_Y + node_dy;
        }
        for (i_ra_no = 0; i_ra_no < ra_step_num + count_area_num; i_ra_no++)
        {
          if (Node[a_i_node].m_Already_Count_intoRA[0] == FALSE)
          {
            Count_Having_Info_Nodes_Num_Into_RA(0, a_i_node, node_X, node_Y, node_Route);
          }
        }
        Node[a_i_node].m_Moved_num++;
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
    }
    return;
  }
}

//ロボットの交差点を過ぎてからの経過時間を受け取り、超過分の処理を行う関数
void Robot_Next_Section(int a_i_node, int node_Moved_num, double bt) // btは交差点を過ぎてからの経過時間
{
  int stop_count; //停止回数のカウンタ
  int num_route;  //何番目の経路の情報なのか
  int next_route; //次の経路
  NodeR[a_i_node].m_On_Intersection = 1; //現在地は交差点上, m_On_Intersectionはロボットが交差点にいる時のフラグ
  num_route = NodeR[a_i_node].m_Num_Route; //ロボットが使う経路の番号
  next_route = Route[num_route].m_Route[node_Moved_num]; //進行方向の情報 Node_DistのGet_Robot_Infoで設定
  Node[a_i_node].m_V = Route[num_route].m_Section_Velocity[node_Moved_num]; //各区間ごとの速度 Node_Dist内，Get_Robot_Infoでテキストファイルから読み取り

  if (next_route == STOP)
  {
    stop_count = NodeR[a_i_node].m_Pause_Count; //ロボットが何回一時停止したか
    Node[a_i_node].m_Vx = 0;
    Node[a_i_node].m_Vy = 0;
    NodeR[a_i_node].m_Pause_Remaining_Time = Route[num_route].m_Pause_Time[stop_count] - bt;
    NodeR[a_i_node].m_Pause_Count++;
    NodeR[a_i_node].m_Pause_Count = NodeR[a_i_node].m_Pause_Count % Route[num_route].m_Num_Pause_Point; //m_Num_Pause_Point 経路中にある停止位置の数
  }

  if (next_route == RIGHT)
  {
    Node[a_i_node].m_Vx = Node[a_i_node].m_V;
    Node[a_i_node].m_Vy = 0;
    Node[a_i_node].m_X += Node[a_i_node].m_Vx * bt;
  }
  if (next_route == LEFT)
  {
    Node[a_i_node].m_Vx = -Node[a_i_node].m_V;
    Node[a_i_node].m_Vy = 0;
    Node[a_i_node].m_X += Node[a_i_node].m_Vx * bt;
  }
  if (next_route == UP)
  {
    Node[a_i_node].m_Vx = 0;
    Node[a_i_node].m_Vy = Node[a_i_node].m_V;
    Node[a_i_node].m_Y += Node[a_i_node].m_Vy * bt;
  }
  if (next_route == DOWN)
  {
    Node[a_i_node].m_Vx = 0;
    Node[a_i_node].m_Vy = -Node[a_i_node].m_V;
    Node[a_i_node].m_Y += Node[a_i_node].m_Vy * bt;
  }

  if (next_route == UP_RIGHT)
  {
    Node[a_i_node].m_Vx = Node[a_i_node].m_V / sqrt(2.0);
    Node[a_i_node].m_Vy = Node[a_i_node].m_V / sqrt(2.0);
    NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X; //m_Robot_Previous_Intersection_X ロボットが一つ前に通過した交差点のX座標
    NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
    Node[a_i_node].m_X += bt * Node[a_i_node].m_Vx;
    Node[a_i_node].m_Y += bt * Node[a_i_node].m_Vy;
  }
  if (next_route == DOWN_RIGHT)
  {
    Node[a_i_node].m_Vx = Node[a_i_node].m_V / sqrt(2.0);
    Node[a_i_node].m_Vy = -Node[a_i_node].m_V / sqrt(2.0);
    NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
    NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
    Node[a_i_node].m_X += bt * Node[a_i_node].m_Vx;
    Node[a_i_node].m_Y += bt * Node[a_i_node].m_Vy;
  }
  if (next_route == UP_LEFT)
  {
    Node[a_i_node].m_Vx = -Node[a_i_node].m_V / sqrt(2.0);
    Node[a_i_node].m_Vy = Node[a_i_node].m_V / sqrt(2.0);
    NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
    NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
    Node[a_i_node].m_X += bt * Node[a_i_node].m_Vx;
    Node[a_i_node].m_Y += bt * Node[a_i_node].m_Vy;
  }
  if (next_route == DOWN_LEFT)
  {
    Node[a_i_node].m_Vx = -Node[a_i_node].m_V / sqrt(2.0);
    Node[a_i_node].m_Vy = -Node[a_i_node].m_V / sqrt(2.0);
    NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
    NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
    Node[a_i_node].m_X += bt * Node[a_i_node].m_Vx;
    Node[a_i_node].m_Y += bt * Node[a_i_node].m_Vy;
  }

  //読んだ経路の数を+1
  Node[a_i_node].m_Moved_num++;
}

//経路を格納した配列を読んでそれに応じてノードを移動させる関数
void Move_Robot(int a_i_node) //Node_Dist内，moveで使用
{
  double node_X, node_Y, node_Vx, node_Vy, node_dx, node_dy, node_Goal_X, node_Goal_Y;
  int node_Moved_num, node_Route;
  int i, j, i_ra_no;
  int num_route;
  double ran;
  double bt; //交差点を過ぎてからの移動時間
  double node_dx_drone;
  double node_dy_drone;
  int flag_drone_over_intersection; //ドローンが交差点を超過したときに立てるフラグ

  node_X = Node[a_i_node].m_X;
  node_Y = Node[a_i_node].m_Y;
  node_Vx = Node[a_i_node].m_Vx;
  node_Vy = Node[a_i_node].m_Vy;
  node_dx = Node[a_i_node].m_Vx * dt; //単位時間にx方向に進む距離
  node_dy = Node[a_i_node].m_Vy * dt; //単位時間にy方向に進む距離
  num_route = NodeR[a_i_node].m_Num_Route; //ロボットが使う経路番号
  Node[a_i_node].m_Moved_num %= Route[num_route].m_Route_Len; //m_Route_Len 経路の区間数
  node_Moved_num = Node[a_i_node].m_Moved_num; //読んだ経路数

  flag_drone_over_intersection = 0;

  //STOP
  if (d_Equal(node_Vx, 0) == 1 && d_Equal(node_Vy, 0) == 1) //x方向の速度，y方向の速度ともに0
  {
    //一時停止がdt以内に終了する場合
    if (NodeR[a_i_node].m_Pause_Remaining_Time <= dt) //dt = 0.1
    {
      bt = dt - NodeR[a_i_node].m_Pause_Remaining_Time; //交差点を過ぎてからの移動時間 = 移動時間ステップ - 一時停止中のロボットの残り一時停止時間
      Robot_Next_Section(a_i_node, node_Moved_num, bt); //ロボットの交差点を過ぎてからの経過時間を受け取り、超過分の処理を行う関数
      NodeR[a_i_node].m_On_Intersection = 0; //m_On_Intersectionはロボットが交差点にいる時のフラグ
    }
    else
    {
      //一時停止がdt以内に終了しない場合
      NodeR[a_i_node].m_Pause_Remaining_Time -= dt;
    }
    return;
  }

  //斜めの移動用
  if (d_Equal(node_Vx, 0) != 1 && d_Equal(node_Vy, 0) != 1)
  {
    //右上向き
    if (node_Vx > 0.0 && node_Vy > 0.0)
    {

      if ((NodeR[a_i_node].m_Robot_Previous_Intersection_X + A_i) <= node_X + node_dx && NodeR[a_i_node].m_On_Intersection != 1) //ロボットが1ステップで次の交差点を超える場合 かつ ロボットが交差点上にいない 
      {
        //交差点を超過したとき
        Node[a_i_node].m_X = NodeR[a_i_node].m_Robot_Previous_Intersection_X + A_i;
        Node[a_i_node].m_Y = NodeR[a_i_node].m_Robot_Previous_Intersection_Y + A_i;
        NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
        NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
        bt = ((node_X + node_dx) - NodeR[a_i_node].m_Robot_Previous_Intersection_X) / node_Vx; //交差点を超過した分の時間を計算
        flag_drone_over_intersection = 1;
      }
      else 
      {
        Node[a_i_node].m_X += node_dx;
        Node[a_i_node].m_Y += node_dy;
        NodeR[a_i_node].m_On_Intersection = 0;
      }
    }
    //右下向き
    if (node_Vx > 0.0 && node_Vy < 0.0)
    {
      if ((NodeR[a_i_node].m_Robot_Previous_Intersection_X + A_i) <= node_X + node_dx && NodeR[a_i_node].m_On_Intersection != 1)
      {
        //交差点を超過したとき
        Node[a_i_node].m_X = NodeR[a_i_node].m_Robot_Previous_Intersection_X + A_i;
        Node[a_i_node].m_Y = NodeR[a_i_node].m_Robot_Previous_Intersection_Y - A_i;
        NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
        NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
        bt = ((node_X + node_dx) - NodeR[a_i_node].m_Robot_Previous_Intersection_X) / node_Vx;
        flag_drone_over_intersection = 1;
      }
      else
      {
        Node[a_i_node].m_X += node_dx;
        Node[a_i_node].m_Y += node_dy;
        NodeR[a_i_node].m_On_Intersection = 0;
      }
    }
    //左上向き
    if (node_Vx < 0.0 && node_Vy > 0.0)
    {
      if ((NodeR[a_i_node].m_Robot_Previous_Intersection_X - A_i) >= node_X + node_dx && NodeR[a_i_node].m_On_Intersection != 1)
      {
        //交差点を超過したとき
        Node[a_i_node].m_X = NodeR[a_i_node].m_Robot_Previous_Intersection_X - A_i;
        Node[a_i_node].m_Y = NodeR[a_i_node].m_Robot_Previous_Intersection_Y + A_i;
        NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
        NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
        bt = (NodeR[a_i_node].m_Robot_Previous_Intersection_X - (node_X + node_dx)) / (-node_Vx);
        flag_drone_over_intersection = 1;
      }
      else
      {
        Node[a_i_node].m_X += node_dx;
        Node[a_i_node].m_Y += node_dy;
        NodeR[a_i_node].m_On_Intersection = 0;
      }
    }
    //左下向き
    if (node_Vx < 0.0 && node_Vy < 0.0)
    {
      if ((NodeR[a_i_node].m_Robot_Previous_Intersection_X - A_i) >= node_X + node_dx && NodeR[a_i_node].m_On_Intersection != 1)
      {
        //交差点を超過したとき
        Node[a_i_node].m_X = NodeR[a_i_node].m_Robot_Previous_Intersection_X - A_i;
        Node[a_i_node].m_Y = NodeR[a_i_node].m_Robot_Previous_Intersection_Y - A_i;
        NodeR[a_i_node].m_Robot_Previous_Intersection_X = Node[a_i_node].m_X;
        NodeR[a_i_node].m_Robot_Previous_Intersection_Y = Node[a_i_node].m_Y;
        bt = (NodeR[a_i_node].m_Robot_Previous_Intersection_X - (node_X + node_dx)) / (-node_Vx);
        flag_drone_over_intersection = 1;
      }
      else
      {
        Node[a_i_node].m_X += node_dx;
        Node[a_i_node].m_Y += node_dy;
        NodeR[a_i_node].m_On_Intersection = 0;
      }
    }
    //交差点超過した時の処理(四方向まとめて)
    if (flag_drone_over_intersection == 1)
    {
      Robot_Next_Section(a_i_node, node_Moved_num, bt);
    }
    return;
  }

  //W->E
  if (node_Vx > 0.0)
  {

    for (i = 0; i <= block_num; i++)
    {
      //交差点を超える
      if (node_X <= A_i * i && A_i * i < node_X + node_dx && NodeR[a_i_node].m_On_Intersection != 1)
      {
        //とりあえず交差点まで移動しておく
        Node[a_i_node].m_X = A_i * i;
        bt = (node_X + node_dx - (A_i * i)) / node_Vx; //交差点を超過した分の時間を計算
        Robot_Next_Section(a_i_node, node_Moved_num, bt);
        break;
      }
    }
    //交差点を超えていない時
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
      NodeR[a_i_node].m_On_Intersection = 0;
    }
    return;
  }

  //E->W
  if (node_Vx < 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_X >= A_i * i && node_X + node_dx < A_i * i && NodeR[a_i_node].m_On_Intersection != 1)
      {
        Node[a_i_node].m_X = A_i * i;
        bt = ((A_i * i) - (node_X + node_dx)) / (-node_Vx);
        Robot_Next_Section(a_i_node, node_Moved_num, bt);
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
      NodeR[a_i_node].m_On_Intersection = 0;
    }
    return;
  }

  //S->N
  if (node_Vy > 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_Y <= A_i * i && node_Y + node_dy > A_i * i && NodeR[a_i_node].m_On_Intersection != 1)
      {
        Node[a_i_node].m_Y = A_i * i;

        bt = ((node_Y + node_dy) - (A_i * i)) / node_Vy;
        Robot_Next_Section(a_i_node, node_Moved_num, bt);
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
      NodeR[a_i_node].m_On_Intersection = 0;
    }
    return;
  }

  //N->S
  if (node_Vy < 0.0)
  {
    for (i = 0; i <= block_num; i++)
    {
      if (node_Y >= A_i * i && node_Y + node_dy < A_i * i && NodeR[a_i_node].m_On_Intersection != 1)
      {
        Node[a_i_node].m_Y = A_i * i;

        bt = ((A_i * i) - (node_Y + node_dy)) / (-node_Vy);
        Robot_Next_Section(a_i_node, node_Moved_num, bt);
        break;
      }
    }
    if (node_Moved_num == Node[a_i_node].m_Moved_num)
    {
      Node[a_i_node].m_X += node_dx;
      Node[a_i_node].m_Y += node_dy;
      NodeR[a_i_node].m_On_Intersection = 0;
    }
    return;
  }
}

//送信可能範囲を設定する関数
//m_X1 < m_X2, m_Y1 < m_Y2 とすること
void Set_Transmittable_Area()
{
  double FS_X = Node[0].m_X;
  double FS_Y = Node[0].m_Y;
  double delta = 25.0; //TA幅の半分
  int i, j;

  //メモリの確保
  TA = (Transmittable_Area *)malloc(sizeof(Transmittable_Area) * ta_num);
  if (TA == NULL)
  {
    puts("memory allocation error ( TA )");
    exit(EXIT_FAILURE);
  }

  i = 0;
  TA[i].m_X1 = -delta;
  TA[i].m_Y1 = d_ta * A_i + D; //d_ta: FSからTAまでの交差点数, D: 交差点と送信可能エリアの距離(125) [m]
  TA[i].m_X2 = delta;
  TA[i].m_Y2 = d_ta * A_i + D + L; //L: 送信可能エリアTAの長さ[m]
  
  for (j = 1; j <= d_ta - 1; j++)
  {
    i++;
    
    //横方向のTAの設定
    if (i > 1 && i % 2 == 1){ 
      TA[i].m_X1 = A_i + D;
      TA[i].m_Y1 = (d_ta - j + 1) * A_i - delta;
      TA[i].m_X2 = A_i + D + L;
      TA[i].m_Y2 = (d_ta - j + 1) * A_i + delta;
    }
    else if(i < 2) { 
      TA[i].m_X1 = D;
      TA[i].m_Y1 = d_ta * A_i - delta;
      TA[i].m_X2 = D + L;
      TA[i].m_Y2 = d_ta * A_i + delta;
    }
    if (i >= 3){ //!!注意!!
      break;
    }
    
    //縦方向のTAの設定
    i++;
    if(i % 2 == 0){
      TA[i].m_X1 = A_i - delta;
      TA[i].m_Y1 = (d_ta - j) * A_i + D;
      TA[i].m_X2 = A_i + delta;
      TA[i].m_Y2 = (d_ta - j) * A_i + D + L;
    }
  } //i = 3
  
  //Y軸対象の位置に追加
  i++;
  for (j = 0; j < i - 1; j++)
  {
    TA[i + j].m_X1 = -TA[j + 1].m_X2;
    TA[i + j].m_Y1 = TA[j + 1].m_Y1;
    TA[i + j].m_X2 = -TA[j + 1].m_X1;
    TA[i + j].m_Y2 = TA[j + 1].m_Y2;
  } //j = 6
  
 
  i += j; //i = 7
  //X軸対称の位置に追加
  for (j = 0; j < i; j++)
  {
    TA[i + j].m_X1 = TA[j].m_X1;
    TA[i + j].m_Y1 = -TA[j].m_Y2;
    TA[i + j].m_X2 = TA[j].m_X2;
    TA[i + j].m_Y2 = -TA[j].m_Y1;
  } 
 
   i += j; 
 
  //y=xの直線に対称の位置に追加
  for (j = 0; j < i; j++)
  {
    TA[i + j].m_X1 = TA[j].m_Y1;
    TA[i + j].m_Y1 = TA[j].m_X1;
    TA[i + j].m_X2 = TA[j].m_Y2;
    TA[i + j].m_Y2 = TA[j].m_X2;
  } 
 
  i += j; 
  
 
  printf("%d\n",i);
  //FSの分だけオフセット(FSを基準点にする)
  for (j = 0; j < i + 1; j++)
  {
    TA[j].m_X1 += FS_X;
    TA[j].m_Y1 += FS_Y;
    TA[j].m_X2 += FS_X;
    TA[j].m_Y2 += FS_Y;
    
    //TAの座標を表示
    // printf("TA[%d].m_X1 = %f\n", j, TA[j].m_X1);
    // printf("TA[%d].m_Y1 = %f\n", j, TA[j].m_Y1);
    // printf("TA[%d].m_X2 = %f\n", j, TA[j].m_X2);
    // printf("TA[%d].m_Y2 = %f\n", j, TA[j].m_Y2);
  }
}



//RAを1つ設定する関数
void Set_Restricted_Area(int ra_no, int d_ra)
{
  int i, j;
  double alpha = A_i / 10.0;
  double FS_X = Node[0].m_X;
  double FS_Y = Node[0].m_Y;
  double delta = A_i / 10.0;

  i = 0;
  RA[ra_no][i].m_X1 = -alpha;
  RA[ra_no][i].m_X2 = alpha;
  RA[ra_no][i].m_Y1 = -A_i * d_ra + delta;
  RA[ra_no][i].m_Y2 = -RA[ra_no][i].m_Y1;

  for (j = 0; j < d_ra - 1; j++){
    i++;
    if (i == 1)
    {
      RA[ra_no][i].m_X1 = A_i * (j + 1) - alpha;
      RA[ra_no][i].m_X2 = A_i * (j + 1) + alpha;
      RA[ra_no][i].m_Y1 = -A_i * (j + 2) + delta;
      RA[ra_no][i].m_Y2 = -RA[ra_no][i].m_Y1;
      
    }
    
    if ( i == 2)
    {
      RA[ra_no][i].m_X1 = A_i * (j + 1) - alpha;
      RA[ra_no][i].m_X2 = A_i * (j + 1) + alpha;
      RA[ra_no][i].m_Y1 = -A_i + delta;
      RA[ra_no][i].m_Y2 = -RA[ra_no][i].m_Y1;
    }
  }
  
  i++;

  //y軸に対称の位置に追加
  for (j = 0; j < d_ra - 1; j++)
  {
    RA[ra_no][i + j].m_X1 = -RA[ra_no][j + 1].m_X2;
    RA[ra_no][i + j].m_Y1 = RA[ra_no][j + 1].m_Y1;
    RA[ra_no][i + j].m_X2 = -RA[ra_no][j + 1].m_X1;
    RA[ra_no][i + j].m_Y2 = RA[ra_no][j + 1].m_Y2;
  }
  i += j;
  
  //y=xの直線に対称の位置に追加
  for (j = 0; j < i; j++)
  {
    RA[ra_no][i + j].m_X1 = RA[ra_no][j].m_Y1;
    RA[ra_no][i + j].m_Y1 = RA[ra_no][j].m_X1;
    RA[ra_no][i + j].m_X2 = RA[ra_no][j].m_Y2;
    RA[ra_no][i + j].m_Y2 = RA[ra_no][j].m_X2;
  }
  i += j; //i = 18
  
  //FSの分だけオフセット
  for (j = 0; j < i; j++)
  {
    RA[ra_no][j].m_X1 += FS_X;
    RA[ra_no][j].m_Y1 += FS_Y;
    RA[ra_no][j].m_X2 += FS_X;
    RA[ra_no][j].m_Y2 += FS_Y;
    // printf("RA[%d][%d].m_X1 = %f\n", ra_no, j, RA[ra_no][j].m_X1);
    // printf("RA[%d][%d].m_Y1 = %f\n", ra_no, j, RA[ra_no][j].m_Y1);
    // printf("RA[%d][%d].m_X2 = %f\n", ra_no, j, RA[ra_no][j].m_X2);
    // printf("RA[%d][%d].m_Y2 = %f\n\n", ra_no, j, RA[ra_no][j].m_Y2);
  }
}
  
  

//Set_Restricted_Area()を呼び出してRAを（ra_step_num + count_area_num）個だけ設定
void set_restricted_area()
{
  int i;
  

  if ((RA = (Restricted_Area **)malloc(sizeof(Restricted_Area *) * (ra_step_num + count_area_num))) == NULL)
  {
    puts("memory allocation error ( RA )");
    exit(EXIT_FAILURE);
  }

  for (i = 0; i < ra_step_num + count_area_num - 1; i++) //ra_step_num =1 count_area_num = 5
  {
    //ra_num[i] = 2 + (d_ra[i] - 1) * 4;
    ra_num[i] = 10;
    //printf("ra_num[%d] = %d\n",i, ra_num[i]);
    
    if ((RA[i] = (Restricted_Area *)malloc(sizeof(Restricted_Area) * ra_num[i])) == NULL)
    {
      puts("memory allocation error ( RA )");
      exit(EXIT_FAILURE);
    }
    //printf("Set_RA(%d, %d)\n",i, d_ra[i]);
    if (i == 0)
    {
       Set_Restricted_Area(i, d_ra[i]);
    }
    
  }
}
