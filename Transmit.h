////////////////////////////
// 情報伝送に関する関数群 //
////////////////////////////

////////////////
// 関数リスト //
//////////////////////////////////////////////////
// transmit()
// flooding()
//////////////////////////////////////////////////

int num_transmit_robot[1000] = {0}; //時間毎にロボットの情報送信回数をカウント


//a_iNodeから送信範囲の全てのノードへ情報を送る関数
//戻り値：送った回数
int transmit(int a_i_node)
{
  int i, j;
  double x_a_iNode = Node[a_i_node].m_X; //a_iNodeの位置
  double y_a_iNode = Node[a_i_node].m_Y;
  double x_jNode; //jNodeの位置
  double y_jNode;

  int num_transmit = 0; //送った回数

  //gotoの代わりの処理
  while (TRUE)
  {
    //a_iNodeがFSの時
    if (d_Equal(Node[a_i_node].m_Vx, 0.0) && d_Equal(Node[a_i_node].m_Vy, 0.0) && a_i_node == 0)
      break;

    //a_iNodeがロボットの時
    if (a_i_node <= num_robot)
    {
      break;
    }

    //送信可能範囲にいるかどうか
    for (i = 0; i < ta_num; i++) //ta_numはTAの数
    {
      if (d_Equal(Node[a_i_node].m_Vx, 0.0) && (y_a_iNode >= TA[i].m_Y1) && (y_a_iNode <= TA[i].m_Y2) && x_a_iNode > TA[i].m_X1 && x_a_iNode < TA[i].m_X2)
      {
       
        break; //x方向の速度が0で，範囲内(TAの中)ならfor文を抜ける
      }
      if (d_Equal(Node[a_i_node].m_Vy, 0.0) && (x_a_iNode >= TA[i].m_X1) && (x_a_iNode <= TA[i].m_X2) && y_a_iNode > TA[i].m_Y1 && y_a_iNode < TA[i].m_Y2)
      {
       
        break; //y方向の速度が0で，範囲内(TAの中)ならfor文を抜ける
      }

      else if (i == ta_num - 1)
        return 0; //範囲外なら終了
    }
    break;
  }

  //全てのノードを辿る
  for (j = 0; j < N_MAX; j++)
  {

    if (j == a_i_node)
      continue;

    //jNodeが存在していなかったら，continue
    if (Node[j].m_Exist == FALSE)
      continue;

    //jNodeが既に情報を持っていたら、continue
    if (Node[j].m_Info == TRUE)
      continue;

    //jNodeの位置
    x_jNode = Node[j].m_X;
    y_jNode = Node[j].m_Y;

    //受信ノードについての位置判定 -> 行わないのでコメントアウト
    //if ( target_w < x_jNode && x_jNode < target_b )
    //  continue;

    if (a_i_node <= num_robot && a_i_node != 0)
    {
      //ロボットの場合
      if (sqrt(pow(fabs(x_a_iNode - x_jNode), 2) + pow(fabs(y_a_iNode - y_jNode), 2)) <= NodeR[a_i_node].m_Transmission_distnace) //関数fabs: 絶対値を計算 ロボットの送信可能距離内にノードがいるなら
      {
        
        Node[j].m_Info = TRUE;
        Node[j].m_Received_From_Robot = TRUE;
        num_transmit++; //送った回数

        for (i = 0; i < Twait_max / 100; i++)
        {
          if (i * 100 < Twait && Twait <= ((i + 1) * 100) + 0.5)
            num_transmit_robot[i]++;
        }
      }
    }
    else
    {
      //それ以外の場合
      //静止ノードから送信する
      //FSから情報をもらったら避けるフラグはTHROUGHにする
      if (d_Equal(Node[a_i_node].m_Vx, 0.0) && d_Equal(Node[a_i_node].m_Vy, 0.0) && fabs(x_a_iNode - x_jNode) <= r && fabs(y_a_iNode - y_jNode) <= r && a_i_node == 0) //FS
      {
        if (USE_FS == 1) //FSを用いるかどうかのフラグ
        {
          Node[j].m_Info = TRUE;
          Node[j].m_Already_Set_Avoid_Flag = TRUE;
          Node[j].m_Received_From_FS = TRUE;
          num_transmit++;
        }
      }
      //a_iNodeとj_Nodeの間の距離がr以下だったら、情報を渡して送った回数をインクリメント
      else if (d_Equal(y_a_iNode, y_jNode) && fabs(x_a_iNode - x_jNode) <= r) //a_iNodeとjNodeのy座標が等しい かつ x座標の差が送信可能距離以下
      {
        Node[j].m_Info = TRUE;
        // if (a_i_node <= num_robot) //ロボットなら
        // {
        //   Node[j].m_Received_From_Robot = TRUE;
        //   for (i = 0; i < Twait_max / 1000; i++)
        //   {
        //     if (i * 1000 < Twait && Twait <= ((i + 1) * 1000) + 0.5)
        //       num_transmit_robot[i]++;
        //   }
        // }
        num_transmit++;
      }
      else if (d_Equal(x_a_iNode, x_jNode) && fabs(y_a_iNode - y_jNode) <= r) //a_iNodeとjNodeのx座標が等しい かつ y座標の差が送信可能距離以下
      {
        Node[j].m_Info = TRUE;
        // if (a_i_node <= num_robot) //ロボットなら
        // {
        //   Node[j].m_Received_From_Robot = TRUE;
        //   for (i = 0; i < Twait_max / 1000; i++)
        //   {
        //     if (i * 1000 < Twait && Twait <= ((i + 1) * 1000) + 0.5)
        //       num_transmit_robot[i]++;
        //   }
        // }
        num_transmit++;
      }
    }
  }

  //送った回数を返す
  return num_transmit;
}

//フラッディングする関数
void flooding()
{
  int i;
  int flooding_end;

  //全てのノードを、まだフラッディングしていないという状態にする
  for (i = 0; i < N_MAX; i++)
  {
    Node[i].m_Already_Flooding = FALSE;
  }

  //無限ループ
  while (TRUE)
  {
    flooding_end = TRUE;

    //全てのノードを辿る
    for (i = 0; i < N_MAX; i++)
    {
      // iNodeが存在していなかったらcontinue
      if (Node[i].m_Exist == FALSE)
        continue;

      //iNodeが情報を持っていなかったら、continue
      if (Node[i].m_Info == FALSE)
        continue;

      //iNodeが既にフラッディング済みであったら、continue
      if (Node[i].m_Already_Flooding == TRUE)
        continue;

      //iNodeを、フラッディング済みという状態にする
      Node[i].m_Already_Flooding = TRUE;

      //1回でも情報を送信したら、フラッディングを終了しない
      if (transmit(i) > 0)
        flooding_end = FALSE;
    }

    if (flooding_end == TRUE)
      break;
  }
}
