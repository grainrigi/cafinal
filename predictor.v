`include "define.v"

/**** 分岐予測器
 過去の分岐結果を保存し、その結果に基づくw_takenの予測を次回の分岐時に提供します。

 * ワイヤの役割
   * 分岐結果保存用
     w_baddr 分岐命令のアドレス
     w_br    分岐したかどうか(w_taken)
     w_bdst  分岐先アドレス
     w_be    分岐結果の保存を次回のposedgeで行うかどうか
   * 予測取得用
     w_paddr 予測したい命令のアドレス
     w_pre   予測を提供できるかどうか
     w_pr    予測結果(w_taken)
     w_pdst  予測した命令の分岐先アドレス
  
  * 予測結果の取得に関して
    分岐予測が必要となる状況をまず考えましょう。

    まず、r_pcが分岐命令を指しているとき、
    w_npcは分岐先の最初の命令を指していることが望ましいです。
    よって、分岐予測を提供する必要があるのはr_pcが分岐命令を指しているときです。

    よって、w_paddrにはr_pcを供給します。
    このとき、分岐結果が保存されていれば、
    w_preは1となり、
    w_prは分岐をtakeするべきかどうか、
    w_pdstにw_tpcの値が供給されます。
    よって、w_npcにはw_prの値に応じてw_pdst(taken)またはw_pc4(not taken)を供給します。

    続いて、予測結果の正しさについて検証する必要が有ります。
    分岐判定はEXステージで行われるので、正しさが判明するのは上述の状態から2サイクル後です。

    w_prの値をIFステージからEXステージまで伝播させます(w_pr -> IfId_pr -> Id2Ex_pr)。
    これにより、w_takenとId2Ex_prを比較すれば予測の成否がわかります。

    予測が失敗していれば直ちに正しい分岐先をさすように修正しなければなりません。
    本来の分岐先は、Id2Ex_tpc(taken), Id2Ex_pc4(not taken)となります。
    予測が失敗した場合w_npcに上述の値を供給します。

    なお、予測が失敗した場合、2サイクルストールします。
    IDステージには実行する必要のない命令が供給されており、
    IFステージでもr_pcが実行する必要のない命令を指してしまっているからです。
    これら2命令に関して、レジスタとメモリへの書き込みを無効化してnop化する必要が有ります。
    (さらに、無効な分岐命令がnpcに干渉しないようにw_opも変更すべきです)

  * 分岐履歴の保存について
    分岐履歴キャッシュは2-wayのLRUキャッシュで、
    履歴自体は2ビットの飽和カウンタで記録されます。

    分岐情報は以下の4状態に分けられます。

    11: Strongly Taken
    10: Weakly Taken
    01: Weakly Not Taken
    00: Strongly Not Taken

    一度の分岐結果の供給ではこの状態を上または下に1つまでしか移動できません。
    よって、何回も分岐が連続でtakenとなっていれば、
    一回not takenになったとしても次の予測結果がすぐにはnot takenに転じることはありません。

    先述の通り、履歴保存スロットは2スロットです。
    履歴には、分岐命令の位置するアドレス、分岐情報、分岐先アドレスが保存されています。
    分岐先アドレスを保存するのはIFステージで(命令のデコード前に)分岐先を決定しなければならないからです。
    スロットがいっぱいになった場合、もっとも昔に書き込まれたスロットの内容が上書きされます。
*/
module m_predictor (w_clk, w_baddr, w_br, w_bdst, w_be, w_paddr, w_pr, w_pdst, w_pre);
  input  wire        w_clk;
  input  wire [`HADDR] w_baddr, w_bdst;
  input  wire        w_br, w_be;
  input  wire [`HADDR] w_paddr;
  output wire        w_pr;
  output wire [`HADDR] w_pdst;
  output wire        w_pre;

  // 履歴の保存領域
  reg [`HADDR] r_addr[0:1];     // 分岐元アドレス(キャッシュのキー)
  reg        r_priority[0:1]; // 各スロットの優先度(0が一番高く、1ならば次の新規書き込みで破棄)
  reg [`HADDR] r_dst[0:1];      // 分岐先アドレス
  reg [1:0]  r_predict[0:1];  // 分岐情報(2ビット飽和カウンタ)
  generate genvar g;
    for (g = 0; g < 2; g = g + 1) begin : Gen
      // 変に反応しないように無効な値を入れておく
      initial r_addr[g] = ~(`HADDR_WIDTH'd0);
      // 優先度が最初から振られてないと誤動作するので
      initial r_priority[g] = g;
      initial r_dst[g] = ~(`HADDR_WIDTH'd0);
      initial r_predict[g] = 0;
    end
  endgenerate

  // 保存先の選択：既にエントリがあればそのインデックス、なければ4
  wire [1:0] w_bselect = (r_addr[0] == w_baddr) ? 0
                       : (r_addr[1] == w_baddr) ? 1
                       : 2;
  // 2ビット幅に縮めたバージョン(selector effective)
  wire [1:0] w_bselect_e = w_bselect[0];
  // 選択された保存先の優先度
  wire [1:0] w_bpriority = r_priority[w_bselect_e];
  // 予測の読み込み元の選択：エントリがヒットすればそのインデックス、なければ4
  wire [1:0] w_pselect = (r_addr[0] == w_paddr) ? 0
                       : (r_addr[1] == w_paddr) ? 1
                       : 2;
  wire [1:0] w_pselect_e = w_pselect[0];
  // 供給された分岐結果を既存の分岐情報に足し込んだ結果(飽和カウンタ)
  wire [1:0] w_npd = r_predict[w_bselect_e][1] == w_br ? {w_br, w_br} :
                     (r_predict[w_bselect_e] + (w_br ? 1 : -1));
  
  // update prediction
  always @(posedge w_clk) if (w_be == 1) begin
    if (w_bselect != 2) begin
      // update priority
      // $write("Update prediction: pc=%x, br=%b, pr=%b\n", w_baddr, w_br, w_npd);
    end
  end
  
  generate genvar g2;
    for (g2 = 0; g2 < 2; g2 = g2 + 1) begin : Gen2
      // 分岐結果の保存処理(w_beが1のときのみ)
      always @(posedge w_clk) if (w_be == 1) begin
        if (w_bselect != 2) begin
          // 過去に分岐情報が保存されているので、それを更新
          // bselectで選択されたスロットならば分岐情報をw_brで更新する
          r_predict[g2] <= #3 w_bselect_e == g2 ? w_npd : r_predict[g2];
          // 優先度を正しく更新(bselectで選択されたスロットを0にして、それ以外は必要に応じて優先度を増加)
          r_priority[g2] <= #3 w_bselect_e == g2 ? 0 : 1;
        end else
          // 分岐情報を新たに書き込む
          if (r_priority[g2] == 1) begin
            // priorityが3のスロットの場合、最も古い情報なので上書き
            r_addr[g2] <= #3 w_baddr;
            r_dst[g2] <= #3 w_bdst;
            r_predict[g2] <= #3 w_br ? 2'b10 : 2'b0;
            // $write("Init prediction: slot=%d, pc=%x, br=%b, pr=%b\n", g2, w_baddr, w_br, w_br ? 2'b10 : 2'b0);
          end
          // 優先度は新たに書き込んだスロットが0、それ以外は1増加
          r_priority[g2] <= #3 (r_priority[g2] == 1) ? 0 : 1;
        end
      end
  endgenerate
  
  // 予測結果の供給(非同期)
  assign w_pr = r_predict[w_pselect_e][1];
  assign w_pdst = r_dst[w_pselect_e];
  assign w_pre = w_pselect != 2;
  
  always @(posedge w_clk) if (w_pre) begin
    //$write("Provide prediction: pc=%x, br=%b, dst=%x\n", w_paddr, w_pr, w_pdst);
  end
endmodule