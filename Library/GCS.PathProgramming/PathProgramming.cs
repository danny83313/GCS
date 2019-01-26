using System;
using System.Collections.Generic;

namespace GCS.PathProgramming
{
    public class Tasks_Assignment
    {
        private int stopiteration = 100;
        private int tabulistlength = 7;
        private double[,] distarray;

        /// <summary>
        /// 停止運算之迭代數(值越大，解越佳，相對運算時間越久)，默認為100代。
        /// </summary>
        public int StopIteration
        {
            set { this.stopiteration = value; }
            get { return this.stopiteration; }
        }

        /// <summary>
        /// 禁忌名單數量，默認為7。
        /// </summary>
        public int TabuListLength
        {
            set { this.tabulistlength = value; }
            get { return this.tabulistlength; }
        }

        /// <summary>
        /// 距離陣列，Row為載具，Column目標目標點。
        /// </summary>
        public double[,] DistArray
        {
            get { return this.distarray; }
        }

        /// <summary>
        /// 載具任務分配最佳化(LoadBalance,載具數量大於等於任務點數量)；
        /// StartPos : 起始點[經,緯,高度] *數量大於等於GoalPos；
        /// GoalPos : 目標點[經,緯,高度]；
        /// </summary>
        public List<double[][]> Planning(List<double[]> StartPos, List<double[]> GoalPos, List<double[]> NoflyPos, ref double[] OptimalValue)
        {
            List<List<int>> save_turning_point = new List<List<int>>();
            List<double[]> ExNoflyPos = new List<double[]>();
            //計算距離陣列
            distarray = CulDistArray(StartPos, GoalPos, NoflyPos, ref ExNoflyPos, ref save_turning_point);
            //規劃
            List<int> Result = TabuSearch(StartPos.Count, GoalPos.Count, distarray, stopiteration, tabulistlength, ref OptimalValue);
            //結果後處理
            List<double[][]> PlanningResult = new List<double[][]>();
            for (int i = 0; i < StartPos.Count; i++)
            {
                //判斷轉折點有沒有在規劃的最佳畫路徑內
                int Target = Result.FindIndex(X => X == i);
                int I = 0;
                bool checkInNoFly = false;
                foreach (var item in save_turning_point)
                {
                    checkInNoFly |= (item[0] == i && item[1] == Target);
                    if (checkInNoFly)
                        break;
                    I++;
                }


                double[][] AUVLocation;
                if (I == save_turning_point.Count)
                    AUVLocation = new double[2][];
                else
                    AUVLocation = new double[2 + save_turning_point[I].Count - 2][];



                AUVLocation[0] = StartPos[i];
                if (checkInNoFly)
                {
                    for (int k = 0; k < save_turning_point[I].Count - 2; k++)
                    {
                        AUVLocation[1 + k] = ExNoflyPos[save_turning_point[I][2 + k]/*轉折點代號*/- (StartPos.Count + GoalPos.Count)];//<------------------轉折點指標要改
                    }
                }
                if (Target + 1 <= GoalPos.Count)
                {
                    AUVLocation[AUVLocation.Length - 1] = GoalPos[Target];
                }
                PlanningResult.Add(AUVLocation);
            }
            return PlanningResult;
        }

        /// <summary>
        /// 計算距離陣列；
        /// StartPos : 起始點[經,緯,高度]；
        /// GoalPos : 目標點[經,緯,高度]；
        /// </summary>
        private double[,] CulDistArray(List<double[]> StartPos, List<double[]> GoalPos, List<double[]> NoflyPos, ref List<double[]> ExNoflyPos, ref List<List<int>> save_turning_point)
        {
            int NoflyCount;
            if (NoflyPos.Count == 0) NoflyCount = NoflyPos.Count;//if haven't no-fly zone
            else
            {//have no-fly zone(avoid amount = -1)
                NoflyCount = NoflyPos.Count - 1;
                ExNoflyPos = exnofly(NoflyPos, 1.2);// calculate turning point*
            }
            int[] astar_ans = new int[NoflyPos.Count];
            double[,] distarray = new double[StartPos.Count, GoalPos.Count];
            for (int i = 0; i < StartPos.Count; i++)
                for (int j = 0; j < GoalPos.Count; j++)
                {
                    bool x = false;

                    // judge the target iine cross no-fly zone or not 
                    for (int k = 0; k < NoflyCount; k++)
                        x = x || crsnofly(StartPos[i][0], StartPos[i][1], GoalPos[j][0], GoalPos[j][1],
                                          NoflyPos[k][0], NoflyPos[k][1], NoflyPos[k + 1][0], NoflyPos[k + 1][1]);

                    // normal distance
                    if (!x)
                        distarray[i, j] = dist(StartPos[i][0], StartPos[i][1], GoalPos[j][0], GoalPos[j][1]);
                    else
                    {// cross no-fly zone 
                        distarray[i, j] = astar(StartPos[i][0], StartPos[i][1], StartPos[i][2], GoalPos[j][0], GoalPos[j][1], GoalPos[j][2], NoflyPos, ExNoflyPos, NoflyCount, StartPos.Count + GoalPos.Count, ref astar_ans); //astar

                        List<int> ForInsertFunction = new List<int>();
                        ForInsertFunction.Add(i);//because cross no-fly zone ,so save this starting point and end point for insert function
                        ForInsertFunction.Add(j);
                        int f = 1;
                        do
                        {
                            f++;
                        } while (!(astar_ans[f] == 0));

                        for (int k = 0; k < f - 1; k++)
                        {// need through which turning point
                            ForInsertFunction.Add(astar_ans[f - 1 - k] + StartPos.Count + GoalPos.Count - 1);
                        }
                        save_turning_point.Add(ForInsertFunction);
                    }
                }
            return distarray;
        }

        /// <summary>
        /// 載具任務分配最佳化(LoadBalance,載具數量大於等於任務點數量)；
        /// StartPointCount : 起始點個數；
        /// GoalPointCount : 目標點個數；
        /// distarray : 每個起點 與 每個目標點 所構成的距離陣列；
        /// StopIteration : 停止運算迭代數(值越大，解越佳，相對運算時間越久)；
        /// TabuListLength : 禁忌名單數量；
        /// </summary>
        private List<int> TabuSearch(int StartPointCount, int GoalPointCount, double[,] distarray, int StopIteration, int TabuListLength, ref double[] OptimalValue)
        {
            //建立初始解(歸一化)
            List<int> Optimal = new List<int>();
            for (int i = 0; i < StartPointCount; i++)
                Optimal.Add(i);
            OptimalValue = new double[2] { 0, 0 };
            //計算目標函數
            for (int k = 0; k < GoalPointCount; k++)
            {
                OptimalValue[1] += distarray[Optimal[k], k];
                if (OptimalValue[0] < distarray[Optimal[k], k])
                    OptimalValue[0] = distarray[Optimal[k], k];
            }
            //定義當前解
            List<int> Current = new List<int>();
            for (int i = 0; i < StartPointCount; i++)
                Current.Add(Optimal[i]);
            //建立禁忌名單
            double[,] TabuList = new double[TabuListLength, 2];
            int TabuListNum = 0;
            //定義停止條件
            int K = 0;
            do
            {
                //定義鄰近最佳解
                List<int> NearOptimal = new List<int>();
                for (int i = 0; i < StartPointCount; i++)
                    NearOptimal.Add(Current[i]);
                double[] NearOptimalValue = new double[2] { double.MaxValue, double.MaxValue };
                //找到鄰近最佳解            //擴展可行解
                for (int i = 0; i < StartPointCount; i++)
                    for (int j = i + 1; j < StartPointCount; j++)
                    {
                        //移步
                        int a = Current[i];
                        int b = Current[j];
                        //定義移步解
                        List<int> move = new List<int>();
                        for (int k = 0; k < StartPointCount; k++)
                            move.Add(Current[k]);
                        move[i] = b;
                        move[j] = a;
                        //計算目標函數
                        double[] NearValue = new double[2] { 0, 0 };
                        for (int k = 0; k < GoalPointCount; k++)
                        {
                            NearValue[1] += distarray[move[k], k];
                            if (NearValue[0] < distarray[move[k], k])
                                NearValue[0] = distarray[move[k], k];
                        }
                        //判斷是否在禁忌名單內
                        bool ok = true;
                        for (int k = 0; k < TabuListLength; k++)
                        {
                            ok &= !(TabuList[k, 0] == NearValue[0] && TabuList[k, 1] == NearValue[1]);
                        }
                        if (ok)
                        {
                            //判斷是否優於鄰近最佳解
                            if (NearValue[0] < NearOptimalValue[0])
                            {
                                //更新鄰近最佳解
                                NearOptimalValue[0] = NearValue[0];
                                NearOptimalValue[1] = NearValue[1];
                                TabuList[TabuListNum, 0] = NearValue[0];
                                TabuList[TabuListNum, 1] = NearValue[1];
                                for (int k = 0; k < StartPointCount; k++)
                                    NearOptimal[k] = move[k];
                            }
                            else if (NearValue[0] == NearOptimalValue[0] && NearValue[1] < NearOptimalValue[1])
                            {
                                //更新鄰近最佳解
                                NearOptimalValue[0] = NearValue[0];
                                NearOptimalValue[1] = NearValue[1];
                                TabuList[TabuListNum, 0] = NearValue[0];
                                TabuList[TabuListNum, 1] = NearValue[1];
                                for (int k = 0; k < StartPointCount; k++)
                                    NearOptimal[k] = move[k];
                            }
                        }
                    }
                //禁忌名單加一，若大於名單大小則設為0
                if (TabuListNum == TabuListLength - 1)
                    TabuListNum = 0;
                else
                    TabuListNum++;
                //更新當前解為鄰近最佳解
                for (int i = 0; i < StartPointCount; i++)
                    Current[i] = NearOptimal[i];
                //判斷是否優於最佳解
                if (NearOptimalValue[0] < OptimalValue[0])
                {
                    //更新鄰近最佳解
                    OptimalValue[0] = NearOptimalValue[0];
                    OptimalValue[1] = NearOptimalValue[1];
                    for (int k = 0; k < StartPointCount; k++)
                        Optimal[k] = NearOptimal[k];
                }
                else if (NearOptimalValue[0] == OptimalValue[0] && NearOptimalValue[1] < OptimalValue[1])
                {
                    //更新鄰近最佳解
                    OptimalValue[0] = NearOptimalValue[0];
                    OptimalValue[1] = NearOptimalValue[1];
                    for (int k = 0; k < StartPointCount; k++)
                        Optimal[k] = NearOptimal[k];
                }
                K++;
            } while (K < StopIteration);//判斷是否到停止條件

            return Optimal;
        }

        private bool crsnofly(double lat0, double lng0, double lat1, double lng1, double lat2, double lng2, double lat3, double lng3)//judge cross or not
        {
            {
                double d1 = crsproduct(lat2, lng2, lat3, lng3, lat0, lng0);//3,4,1
                double d2 = crsproduct(lat2, lng2, lat3, lng3, lat1, lng1);//3,4,2
                double d3 = crsproduct(lat0, lng0, lat1, lng1, lat2, lng2);//1,2,3
                double d4 = crsproduct(lat0, lng0, lat1, lng1, lat3, lng3);//1,2,4

                if (d1 * d2 < 0 && d3 * d4 < 0) return true; //cross

                else if (d1 == 0 && sameline(lat2, lng2, lat3, lng3, lat0, lng0)) return true;//cross

                else if (d2 == 0 && sameline(lat2, lng2, lat3, lng3, lat1, lng1)) return true;//cross

                else if (d3 == 0 && sameline(lat0, lng0, lat1, lng1, lat2, lng2)) return true;//cross

                else if (d4 == 0 && sameline(lat0, lng0, lat1, lng1, lat3, lng3)) return true;//cross

                else return false; //no cross
            }
        }

        private double crsproduct(double lat0, double lng0, double lat1, double lng1, double noflylat0, double noflylng0) //Cross product
        {
            return ((noflylat0 - lat0) * (lng1 - lng0)) - ((lat1 - lat0) * (noflylng0 - lng0));// Cross product array simplify
        }

        private bool sameline(double lat0, double lng0, double lat1, double lng1, double noflylat0, double noflylng0) //same line (overlap)
        {
            double minx = Math.Min(lat0, lat1);
            double maxx = Math.Max(lat0, lat1);
            double miny = Math.Min(lng0, lng1);
            double maxy = Math.Max(lng0, lng1);

            return noflylat0 >= minx && noflylat0 <= maxx && noflylng0 >= miny && noflylng0 <= maxy; //return true or false
        }

        private double astar(double lat0, double lng0, double alt0, double lat1, double lng1, double alt1, List<double[]> NoflyPos, List<double[]> ExNoflyPos, int nflatLength, int latLength, ref int[] astar_ans) // A*
        {// i: start & end piont  / o: distance of dodge no-fly zone

            List<double[]> dll_turnlist = new List<double[]>();

            dll_turnlist.Add(new double[] { lat0, lng0, alt0 });//start point
            for (int i = 0; i < nflatLength; i++)
            {
                dll_turnlist.Add(new double[] { ExNoflyPos[i][0], ExNoflyPos[i][1] });//turning point
            }
            dll_turnlist.Add(new double[] { lat1, lng1, alt1 });//end point

            //

            double[,] cij = new double[nflatLength + 2, nflatLength + 2];
            double[] tlat = new double[nflatLength + 2];//(start, turning0,,,, end)
            double[] tlng = new double[nflatLength + 2];
            for (int i = 0; i < dll_turnlist.Count; i++)
            {
                tlat[i] = dll_turnlist[i][0];
                tlng[i] = dll_turnlist[i][1];
            }

            cij = smalldistance(tlat, tlng, NoflyPos, nflatLength);//small distance array

            //
            double[] h = new double[cij.GetLength(0)];
            for (int i = 0; i < cij.GetLength(0); i++)//H.function
            {
                h[i] = dist(tlat[tlat.Length - 1], tlng[tlng.Length - 1], tlat[i], tlng[i]);//H.Funtion
                if (h[i] == double.MaxValue) h[i] = 0;//if goal, make H.function = 0
            }

            //
            double[,] open = new double[nflatLength * nflatLength + latLength, nflatLength * nflatLength + latLength];
            double[,] close = new double[nflatLength * nflatLength + latLength, nflatLength * nflatLength + latLength];
            double[,] de = new double[nflatLength * nflatLength + latLength, nflatLength * nflatLength + latLength];

            //////////////////////////////search target//////////////////////////////////////
            int C = int.MaxValue;
            double[] C_info = new double[4];
            bool reach_target = false;
            double abs = 0;
            double astar_g = 0;
            int asg = 0;

            do
            {
                double f = double.MaxValue;
                int mini_index = int.MaxValue;
                bool noloop = true;

                for (int i = 0; i < open.GetLength(0); i++) //save open.array [i,3] 
                {
                    if (open[i, 3] < f)  // get F.function min
                    {
                        mini_index = i;
                        f = open[i, 3];
                    }
                }

                for (int i = 0; i < 4; i++) // take f.min row from open.array
                {
                    C_info[i] = open[mini_index, i];
                }
                open = delete(open, mini_index);// delete open C row
                C = Convert.ToInt32(C_info[0]);

                double[,] m = new double[0, 4];

                for (int i = 0; i < cij.GetLength(0); i++) // {open,close,de,C_info}
                {
                    bool a = false;
                    for (int j = 0; j < de.GetLength(0); j++) // dead end 
                    {
                        a |= (de[j, 0] == i);
                    }
                    if (a)
                    { }
                    else
                    {
                        bool b = false;
                        for (int j = 0; j < close.GetLength(0); j++) //close
                        {
                            b |= (close[j, 0] == i);
                        }
                        if (b)
                        { }
                        else
                        {
                            bool c = false;
                            int loop = 0;
                            for (int j = 0; j < open.GetLength(0); j++) //open
                            {
                                c |= (open[j, 0] == i);
                                if (open[j, 0] == i) loop = j;//open current point overlap -> keep point.j in loop
                            }
                            if (c)
                            {
                                if (cij[C, i] + C_info[2] + h[i] < open[loop, 3]) //if overlap , retain better one (F < open.F)
                                {
                                    open[loop, 0] = i;
                                    open[loop, 1] = C;
                                    open[loop, 2] = cij[C, i] + C_info[2];
                                    open[loop, 3] = cij[C, i] + C_info[2] + h[i];
                                    noloop = false;
                                }
                            }

                            //normal
                            else
                            {
                                if (cij[C, i] == double.MaxValue)
                                { }
                                else
                                {
                                    double[] EX_info = new double[] { i, C, cij[C, i] + C_info[2], cij[C, i] + C_info[2] + h[i] }; //{current point,origin point,G.function,F.function}
                                    m = into(m, EX_info);
                                }
                            }
                        }
                    }
                }

                if ((m.GetLength(0) == 0) && (noloop))// if open can be enlarging and doesn't in loop , C_info add to de)
                {
                    de = into(de, C_info);
                }
                else
                {
                    close = into(close, C_info);
                    for (int i = 0; i < m.GetLength(0); i++)
                    {
                        double[] EX_infor = new double[4];
                        for (int j = 0; j < 4; j++)
                        {
                            EX_infor[j] = m[i, j];
                        }
                        open = into(open, EX_infor);
                    }
                }

                //stop situation
                reach_target = false;
                for (int i = 0; i < open.GetLength(0); i++)
                {
                    reach_target |= (open[i, 0] == tlat.Length - 1);//stop situation is arrive target point
                    if (open[i, 0] == tlat.Length - 1)
                    {
                        abs = open[i, 1];
                        asg = i;
                    }
                }
                astar_g = open[asg, 2];// G.function after through turning point (total travel distance)
            } while (!reach_target);

            // when goal , trace travel can get path
            double[] ans = new double[] { tlat.Length - 1, abs };

            do
            {
                int i_close = 0;
                while (!(close[i_close, 0] == abs))
                {
                    i_close++;
                }
                Array.Resize(ref ans, ans.Length + 1);
                ans[ans.Length - 1] = close[i_close, 1];
                abs = close[i_close, 1];

            } while (!(abs == 0));

            for (int i = 0; i < ans.Length; i++)
            {
                astar_ans[i] = Convert.ToInt32(ans[i]);//
            }

            return astar_g;
        }

        private List<double[]> exnofly(List<double[]> NoflyPos, double exnofly_gain)//the turning point after enlarging the no-fly zone
        {
            List<double[]> ExpendNoflyPos = new List<double[]>();
            double pclat = 0;
            double pclng = 0;
            foreach (var point in NoflyPos)
            {
                pclat += (point[0] * Math.PI) / 180;
                pclng += (point[1] * Math.PI) / 180;
            }
            pclat = (pclat / (NoflyPos.Count)) * 180 / Math.PI;
            pclng = (pclng / (NoflyPos.Count)) * 180 / Math.PI;
            for (int i = 0; i < NoflyPos.Count; i++)
            {
                ExpendNoflyPos.Add(new double[] { pclat + exnofly_gain * (NoflyPos[i][0] - pclat), pclng + exnofly_gain * (NoflyPos[i][1] - pclng) });// pc + exnofly_gain*(pi-pc)
            }
            return ExpendNoflyPos;
        }

        static private double dist(double lat0, double lng0, double lat1, double lng1)//distance formula
        {// i: any two point / o: distance
            double X = Math.Cos(lat0 * Math.PI / 180) * Math.Cos(lng0 * Math.PI / 180) -
                       Math.Cos(lat1 * Math.PI / 180) * Math.Cos(lng1 * Math.PI / 180);

            double Y = Math.Cos(lat0 * Math.PI / 180) * Math.Sin(lng0 * Math.PI / 180) -
                       Math.Cos(lat1 * Math.PI / 180) * Math.Sin(lng1 * Math.PI / 180);

            double Z = Math.Sin(lat0 * Math.PI / 180) - Math.Sin(lat1 * Math.PI / 180);

            double Tij = 2 * 6378137 * Math.Asin(Math.Sqrt(Math.Pow(X, 2) +
                                                           Math.Pow(Y, 2) +
                                                           Math.Pow(Z, 2)) / 2); // Great-circle distance formula (Equatorial radius = 6378137m)
            // 2*6378137*sin^-1(√(X^2+Y^2+Z^2)/2)
            if (Tij == 0) return double.MaxValue;

            else return Tij;
        }

        private double[,] smalldistance(double[] slat, double[] slng, List<double[]> NoflyPos, int nflatLength)//small distance array for astart program(starting point,turning point,target point)
        {
            double[,] cij = new double[nflatLength + 2, nflatLength + 2];

            for (int i = 0; i < nflatLength + 2; i++)
            {
                for (int j = 0; j < nflatLength + 2; j++)
                {
                    bool xx = false;
                    for (int k = 0; k < nflatLength; k++)
                    {
                        xx = xx || crsnofly(slat[i], slng[i], slat[j], slng[j], NoflyPos[k][0], NoflyPos[k][1], NoflyPos[k + 1][0], NoflyPos[k + 1][1]);
                    }

                    if (xx == false) cij[i, j] = dist(slat[i], slng[i], slat[j], slng[j]);

                    else cij[i, j] = double.MaxValue;
                }
            }
            return cij;
        }

        private double[,] delete(double[,] array, int c) //take out and delete the row for astar program delete open.function
        {
            double[,] array1 = new double[array.GetLength(0), array.GetLength(1)];
            Array.Copy(array, array1, array.Length);

            array = new double[array1.GetLength(0) - 1, 4];
            int h = 0;
            for (int i = 0; i < array.GetLength(0) + 1; i++)
            {
                if (i != c)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        array[i - h, j] = array1[i, j];
                    }
                }
                else h++;
            }
            return array;
        }

        private double[,] into(double[,] array2d, double[] array1d) // add row for astar program open.function,close.function,and dead-end.function 
        {
            double[,] array1 = new double[array2d.GetLength(0), array2d.GetLength(1)];
            Array.Copy(array2d, array1, array2d.Length);

            array2d = new double[array1.GetLength(0) + 1, 4];

            for (int i = 0; i < array1.GetLength(0); i++)
            {
                for (int j = 0; j < array2d.GetLength(1); j++)
                {
                    array2d[i, j] = array1[i, j];
                }
            }
            for (int i = 0; i < array2d.GetLength(1); i++)
            {
                array2d[array2d.GetLength(0) - 1, i] = array1d[i];
            }
            return array2d;
        }

    }
}
