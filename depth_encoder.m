clear;


bag = rosbag('2022-07-05-18-47-58.bag');    %bagにbagファイルのデータを入れている
% bag = rosbag('2022-07-05-18-50-39.bag');
% bag = rosbag('2022-07-05-18-52-56.bag');
% bag = rosbag('2022-07-05-18-55-16.bag');
% bag = rosbag('2022-07-05-18-58-09.bag');


bSel_depth = select(bag,'Topic','/Depth_path');%bagファイルの中にdepth_pathという名前のtopicをとってくる
msgStructs_depth = readMessages(bSel_depth,'DataFormat','struct');%bselにはアドレスが記録されているからそれをもとにデータをとってくる

bSel_odom = select(bag,'Topic','/robot1/odom');
msgStructs_odom = readMessages(bSel_odom,'DataFormat','struct');

bSel_cmd = select(bag,'Topic','/robot1/mobile_base/commands/velocity');
msgStructs_cmd = readMessages(bSel_cmd,'DataFormat','struct');

L = 6;

% t_str = append(num2str(msgStructs_odom{1, 1}.Header.Stamp.Sec), '.', num2str(msgStructs_odom{1, 1}.Header.Stamp.Nsec));
% t0 = str2double(t_str);
%t0 = msgStructs_odom{1, 1}.Header.Stamp.Sec + msgStructs_odom{1, 1}.Header.Stamp.Nsec * 10^-9;
t0 = bag.MessageList.Time(1);

time_odom = [];
posY = [];
for fr_odom = 1:numel(msgStructs_odom)%numelとlengthは一緒の返り値だがnumelは配列の中身は数値以外でもいい。とにかくnumelを使っておけば問題ない
    %t_str = append(num2str(msgStructs_odom{fr_odom, 1}.Header.Stamp.Sec), '.', num2str(msgStructs_odom{fr_odom, 1}.Header.Stamp.Nsec));
    %t = str2double(t_str);
    %t = msgStructs_odom{fr_odom, 1}.Header.Stamp.Sec + msgStructs_odom{fr_odom, 1}.Header.Stamp.Nsec * 10^-9;
    t = bSel_odom.MessageList.Time(fr_odom);
    time_odom(fr_odom) = t - t0;
    
    posY(fr_odom) = msgStructs_odom{fr_odom, 1}.Pose.Pose.Position.X - msgStructs_odom{1, 1}.Pose.Pose.Position.X;
    
end

time_depth = [];
depth = [];
for fr_depth = 1:numel(msgStructs_depth)
    time_depth(fr_depth) = bSel_depth.MessageList.Time(fr_depth) - t0;
    for i = 1:1
        depth(fr_depth,i) = msgStructs_depth{fr_depth, 1}.Depth(1+i).Data/1000;
    end
end

for fr_cmd = 1:numel(msgStructs_cmd)
    time_cmd(fr_cmd) = bSel_cmd.MessageList.Time(fr_cmd) - t0;
    vel(fr_cmd) = msgStructs_cmd{fr_cmd, 1}.Linear.X;
    ang(fr_cmd) = msgStructs_cmd{fr_cmd, 1}.Angular.Z;
end

time_odom_sync = [];
posY_sync = [];
for i = 1:length(time_depth)
    index = knnsearch(time_odom', time_depth(i)); %knnsearchは一致した要素番号を返す
    time_odom_sync(i) = time_odom(index);
    posY_sync(i) = posY(index);
end

removed_depth = [];
%移動平均コマンドによるノイズ除去(引数に配列とwindow数を設定)
% removed_depth = movmean(depth,10);

%移動平均法によるノイズ除去(window=3)
% for i=1:length(posY_sync)
%     if(i == 1)
%         removed_depth(i) = (depth(i)+depth(i+1))/2;
%     elseif(i == length(posY_sync))
%         removed_depth(i) = (depth(i-1)+depth(i))/2;
%     else
%         removed_depth(i) = (depth(i-1)+depth(i)+depth(i+1))/3;
%     end
% end

% %移動平均によるノイズ除去(wimdow=5)
% for i=1:length(posY_sync)
%     if(i == 1)
%         removed_depth(i) = (depth(i)+depth(i+1)+depth(i+2))/3;
%     elseif(i == 2)
%         removed_depth(i) = (depth(i-1)+depth(i)+depth(i+1)+depth(i+2))/4;
%     elseif(i == length(posY_sync)-1)
%         removed_depth(i) = (depth(i-2)+depth(i-1)+depth(i)+depth(i+1))/4;
%     elseif(i == length(posY_sync))
%         removed_depth(i) = (depth(i-2)+depth(i-1)+depth(i))/3;
%     else
%         removed_depth(i) = (depth(i-2)+depth(i-1)+depth(i)+depth(i+1)+depth(i+2))/5;
%     end
% end

%LPFによるノイズ除去
%深度値は離散データだが、今回はサンプリング周波数が高いため連続データとみなすことができる。そうすることによってサンプリング周波数を使わないでLPFをかけることができる
f = 0.01;
T = 1/(2*pi*f);
for i=1:length(posY_sync)
   if(i == 1)
       removed_depth(i) = depth(i);
   else
       removed_depth(i) = ((T-1)*removed_depth(i-1)+depth(i-1))/T;
   end
end


for i = 1:length(posY_sync)
    L_x(i) = L - posY_sync(i);
%     err(i) = depth(i) - L_x(i);    %フィルタをかける前のデータで誤差を取る
    err(i) = removed_depth(i) - L_x(i);     %フィルタをかけた後のデータで誤差を取る
end

time_cmd_sync = [];
vel_sync = [];
ang_sync = [];
for i = 1:length(time_depth)
    index = knnsearch(time_cmd', time_depth(i));
    time_cmd_sync(i) = time_cmd(index);
    vel_sync(i) = vel(index);
    ang_sync(i) = ang(index);
    
end


%速度命令が0になったときより先のデータは消す
%extはextract（引き抜く）
for i = 1:length(err)
    if vel_sync(i) > 0
        err_ext(i) = err(i);
    else
        break
    end
end



removed_depth_ext = [];
L_x_ext = [];
vel_sync_ext = [];
time_depth_ext = [];
time_odom_ext = [];
time_odom_sync_ext = [];
time_cmd_sync_ext = [];
for i = 1:length(err_ext)
    removed_depth_ext(i) = removed_depth(i);
    L_x_ext(i) = L_x(i);
    vel_sync_ext(i) = vel_sync(i);
    time_depth_ext(i) = time_depth(i);
    time_odom_ext(i) = time_odom(i);
    time_odom_sync_ext(i) = time_odom_sync(i);
    time_cmd_sync_ext(i) = time_cmd_sync(i);
end

% %深度値のサンプリング時間とサンプリング周波数を計算
for i=1:length(time_depth_ext)-1
    subtraction_time_depth(i) = time_depth(i+1) - time_depth(i);
    frequency_time_depth(i) = 1/subtraction_time_depth(i);
end

frequency_average = mean(frequency_time_depth);

%ホイールエンコーダの値が0.5[m]ずつ区切った時のグラフを作成する

%structで宣言することによって、配列の中に可変型の配列を入れることができる。こうすることで一元化できる
err_divide = struct();

%今回は0.5[m]で区切るため以下のように定義
interval_arr = [5.5, 5, 4.5, 4, 3.5, 3, 2.5, 2, 1.5, 1];

%初期化
for i = 1 : length(interval_arr)
    err_divide(i).err = [];
    err_divide(i).interval = interval_arr(i);
    err_divide(i).index = 1;
end

for i = 1 : length(err_ext)
    
    for j = 1:length(interval_arr)
        
        interval = err_divide(j).interval;%ドットの後ろで配列の要素を指定
        index = err_divide(j).index;
        
        %ここでボーダーより大きいか判断している。interval_arrは昇順になっているので自動的に分けることができる
        if L_x_ext(i) > interval
            
            %ここでマッチしたデータを配列に入れている
            err_divide(j).err(index) = err_ext(i);
            
            %データ番号が追えるようにindexに入れておく
            err_divide(j).index = index + 1;
            
            %もし見つけたら二つ目のループを抜け出す
            break;
        end
        
    end
    
end
    


%負の遺産,区切った時のヒストグラム作成に作ったやつ

% value_60 = 1;
% value_55 = 1;
% value_50 = 1;
% value_45 = 1;
% value_40 = 1;
% value_35 = 1;
% value_30 = 1;
% value_25 = 1;
% value_20 = 1;
% value_15 = 1;
% 
% value_array_err_60 = [];
% value_array_err_55 = [];
% value_array_err_50 = [];
% value_array_err_45 = [];
% value_array_err_40 = [];
% value_array_err_35 = [];
% value_array_err_30 = [];
% value_array_err_25 = [];
% value_array_err_20 = [];
% value_array_err_15 = [];
% 
% for i = 1:length(err_ext)
%     if(L_x_ext(i) >= 5.5)
%         err_divide(1).err = err_ext(i);
%         value_60 = value_60 + 1;
%     elseif(L_x_ext(i) < 5.5 && L_x_ext(i) >= 5.0)
%         value_array_err_55(value_55) = err_ext(i);
%         value_55 = value_55 + 1;
%     elseif(L_x_ext(i) < 5.0 && L_x_ext(i) >= 4.5)
%         value_array_err_50(value_50) = err_ext(i);
%         value_50 = value_50 + 1;
%     elseif(L_x_ext(i) < 4.5 && L_x_ext(i) >= 4.0)
%         value_array_err_45(value_45) = err_ext(i);
%         value_45 = value_45 + 1;
%     elseif(L_x_ext(i) < 4.0 && L_x_ext(i) >= 3.5)
%         value_array_err_40(value_40) = err_ext(i);
%         value_40 = value_40 + 1;
%     elseif(L_x_ext(i) < 3.5 && L_x_ext(i) >= 3.0)
%         value_array_err_35(value_35) = err_ext(i);
%         value_35 = value_35 + 1;
%     elseif(L_x_ext(i) < 3.0 && L_x_ext(i) >= 2.5)
%         value_array_err_30(value_30) = err_ext(i);
%         value_30 = value_30 + 1;
%     elseif(L_x_ext(i) < 2.5 && L_x_ext(i) >= 2.0)
%         value_array_err_25(value_25) = err_ext(i);
%         value_25 = value_25 + 1;
%     elseif(L_x_ext(i) < 2.0 && L_x_ext(i) >= 1.5)
%         value_array_err_20(value_20) = err_ext(i);
%         value_20 = value_20 + 1;
%     elseif(L_x_ext(i) < 1.5 && L_x_ext(i) >= 1.0)
%         value_array_err_15(value_15) = err_ext(i);
%         value_15 = value_15 + 1;
%     end
% end




%%%%%%%%%%%%%%%%%ここからはグラフ表示させるコマンド%%%%%%%%%%%%%%%%%%

% hold on;
% plot(time_odom,posY)
% plot(time_depth,depth(:,1));

%速度命令が0になった時のデータを消去した場合のヒストグラム作成・各区間でのヒストグラム作成
% h = histogram(err_ext);
% h = histogram(value_array_err_60);
% h = histogram(value_array_err_55);
% h = histogram(value_array_err_50);
% h = histogram(value_array_err_45);
% h = histogram(value_array_err_40);
% h = histogram(value_array_err_35);
% h = histogram(value_array_err_30);
% h = histogram(value_array_err_25);
% h = histogram(value_array_err_20);
%区切った時のヒストグラム作成,（ ）の値を変える
% h = histogram(err_divide(10).err);
% xlabel("誤差[m]","FontSize",12);
% ylabel("データ個数[個]","FontSize",12);
% h.BinWidth = 0.005;
% set(gca,"FontSize",12);

%ヒストグラム作成
% h = histogram(err);
% xlabel("誤差[m]","FontSize",12);
% ylabel("データ個数[個]","FontSize",12);
% h.BinWidth = 0.05;
% set(gca,"FontSize",12);

%時間に対してのグラフ作成
% hold on;
% % plot(time_depth,depth, LineWidth = 1.5);
% plot(time_depth,removed_depth, LineWidth = 1.5);
% plot(time_odom_sync,L_x, LineWidth = 1.5);
% xlabel("時間 [s]");
% ylabel("深度値 [m], エンコーダー値 [m]");
% yyaxis right;
% ylabel("誤差 [m], 速度指令値 [m/s]");
% plot(time_odom_sync,err, LineWidth = 1.5);
% plot(time_cmd_sync,vel_sync, LineWidth = 1.5);
% legend("深度値", "エンコーダー値", "誤差", "速度指令値");
% set(gca,"FontSize",12);

%速度命令が0になった時のデータを消去した場合のグラフ作成
% hold on;
% %plot(time_depth,depth, LineWidth = 1.5);
% plot(time_depth_ext,removed_depth_ext, LineWidth = 1.5);
% plot(time_odom_sync_ext,L_x_ext, LineWidth = 1.5);
% xlabel("時間 [s]");
% ylabel("深度値 [m], エンコーダー値 [m]");
% yyaxis right;
% ylabel("誤差 [m], 速度指令値 [m/s]");
% plot(time_odom_sync_ext,err_ext, LineWidth = 1.5);
% plot(time_cmd_sync_ext,vel_sync_ext, LineWidth = 1.5);
% legend("深度値", "エンコーダー値", "誤差", "速度指令値");
% set(gca,"FontSize",12);

% bunsan = var(err_ext);
% hhensa = std(err_ext);
% saihin = mode(err_ext);
% average = mean(err_ext);
% result = [bunsan,hhensa,saihin,average];


%0.5[m]で区切った時の分散・標準偏差・最頻値・平均を各々計算し、一つの表に表示する
%struct型で宣言することによって行と列の個数が違う配列を作ることができる
result = struct();

%初期化
for i = 1 : 10
    result(i).interval = interval_arr(i);
    result(i).bunsan = [];
    result(i).hhensa = [];
    result(i).saihin = [];
    result(i).average = [];
end


%実際に代入
for i = 1 : 10
    result(i).bunsan = var(err_divide(i).err);
    result(i).hhensa = std(err_divide(i).err);
    result(i).saihin = mode(err_divide(i).err);
    result(i).average = mean(err_divide(i).err);
end


%正規確率プロットのグラフ表示（正規分布になっているか確かめるために使う）
normplot(err_divide(10).err);
xlabel("誤差[m]","FontSize",12);

