using System;
using System.Reflection.Metadata;
using ScottPlot.Interactivity.UserActionResponses;
using Line = SegmentBuilder.Coord[];
using DiscreteRobotImplementation;
using ScottPlot.AxisPanels;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using Avalonia.Win32;
using System.Diagnostics;
using Avalonia.Input.Raw;
using System.Dynamic;

namespace SegmentBuilder;

class Coord: ICloneable, IEquatable<Coord>
{
    public double X, Y;

    public Coord(){}
    
    public Coord(double[] XY) => (X, Y) = (XY[0], XY[1]);

    public double[] ToDoubleArray => [X, Y];

    public bool Equals(Coord SomeCoord)
    {
        if(SomeCoord is not null)
        {
            if(SomeCoord is null) return false;
            return X == SomeCoord.X && Y == SomeCoord.Y;
        }
        return false;     
    }

    public override int GetHashCode() => (X, Y).GetHashCode();

    public object Clone() => new Coord(){X = X, Y = Y};
    
    public Coord Copy{private set{} get => (Coord)Clone();}
    
    public static Coord operator *(Coord A, double alpha) => new(){X = A.X * alpha, Y = A.Y * alpha};

    public static Coord operator *(double alpha, Coord A) => new(){X = A.X * alpha, Y = A.Y * alpha};

    public static Coord operator +(Coord A, Coord B) => new(){X = A.X + B.X, Y = A.Y + B.Y};

    public static Coord operator -(Coord A, Coord B) => A + (B * -1);

    public static double operator *(Coord A, Coord B) => A.X * B.X + A.Y * B.Y;

    public double Norm{private set{} get => Math.Sqrt(X * X + Y * Y);}

    public override string ToString() => $"({Math.Round(X, 3)}; {Math.Round(Y, 3)})";
}

class RobotModel
{
    private DataContext Ctx;

    //набор констант робота
    public const double rad = 76.0, hx = 31.0, hy = 44.0,
    f1a = 0.06, f0x = 41.0, f1x = 41.0, c1x = 15.0, c1y = 74.5,
    f2x = 25.0, f2y = 27.8, c2x = 18.0, c2y = 46.0, l1_min = 67.0,
    l1_max = 183.8, l0_min = 62.3, l0_max = 269.6, phi_step = 0.0001,
    l_step = 0.04, flat_width = 792, flat_height = 540;

    public static Coord EvalLine(Line l, double t) => new Coord([l[1].X + t * l[0].X, l[1].Y + t * l[0].Y]);

    public static Coord Compl(Coord l) => new([l.Y, -l.X]);

    public static Coord Ein(double phi) => new([Math.Cos(phi), Math.Sin(phi)]);

    public static Coord Aff(Coord e, double t, Coord q) => new([q.X + t * e.X, q.Y + t * e.Y]);

    public static double dist2(Coord q0, Coord q1) => (q0 - q1).Norm;

    public static double AngNorm(double x) => x < 0.0 ? x + 2 * Math.PI : x;

    public class RobHand: ICloneable
    {
        public Line HandLine;

        public Coord HandPos;

        public Coord HandLineLikeVector => HandLine[0] - HandLine[1];

        public double HandLen;

        public override string ToString() => $"HandLine0 = {HandLine[0]}; HandLine1 = {HandLine[1]}; HandPos = {HandPos}; HandLen = {Math.Round(HandLen, 3)}";

        public double AbsDistance => dist2(HandLine[1], EvalLine(HandLine, HandLen));

        public object Clone() => new RobHand(){HandLine = [HandLine[0], HandLine[1]], HandPos = new([HandPos.X, HandPos.Y]), HandLen = HandLen};

        public RobHand Copy => Clone() as RobHand;
    }

    public class RobState: ICloneable
    {
        public Coord F2, C;

        public RobHand[] Hands = new RobHand[3];

        public override string ToString() => $"F2 = {F2};\n C = {C};\n H0 = {Hands[0]};\n H1 = {Hands[1]};\n HH = {Hands[2]}";

        public void SetRobHand(RobHand H, string HandIndex)
        {
            switch(HandIndex)
            {
                case "h0":
                    Hands[0] = H.Copy;
                    break;
                case "h1":
                    Hands[1] = H.Copy;
                    break;
                case "h":
                    Hands[2] = H.Copy;
                    break;
                default: throw new Exception("НЕИЗВЕСТНЫЙ ИНДЕКС ПАЛЬЦА");
            }
        }

        public double Angle 
        {
            get => Math.Acos(Hands[1].HandLine[0].X);
        }

        public double Angle0 => Math.Atan2(Hands[0].HandLine[0].Y, Hands[0].HandLine[0].X);

        public bool AngleIsPossible
        {
            get
            {
                var CF1 = C - Hands[2].HandPos;
                var angle = Math.Atan2(CF1.Y, CF1.X);
                return angle is not double.NaN && AngNorm(angle) > Math.PI / 4 && AngNorm(angle) < 3 * Math.PI / 4;
            }
        }

        public bool AngleIsPossible1
        {
            get
            {
                var CF1 = C - Hands[2].HandPos;
                var angle = Math.Atan2(CF1.Y, CF1.X);
                return angle is not double.NaN && angle < -Math.PI / 4 && angle > -3 * Math.PI / 4;
            }
        }
        
        public object Clone() => new RobState(){F2 = new([F2.X, F2.Y]), C = new([C.X, C.Y]), Hands = [Hands[0].Copy, Hands[1].Copy, Hands[2].Copy]};

        public RobState Copy => Clone() as RobState;

        public bool CheckState(string arg)
        {
            if(this is null) return false;
            switch(arg)
            {
                case "H":
                    var lh_max = l1_max + hy;
                    var lh_min = l1_min + hy;
                    return Hands[2].AbsDistance < lh_max && Hands[2].AbsDistance > lh_min;
                    break;
                case "F1":
                    return Hands[1].AbsDistance < l1_max && Hands[1].AbsDistance > l1_min;
                    break;
                case "F0F1":
                    var b1 = (Hands[1].AbsDistance < l1_max) && (Hands[1].AbsDistance > l1_min);
                    var b0 = (Hands[0].AbsDistance < l0_max) && (Hands[0].AbsDistance > l0_min);
                    return b0 && b1;
                default: throw new Exception("НЕИЗЕВЕСТНЫЙ ПАРАМЕТР ПРОВЕРКИ"); //здесь может потребоваться дополнительная информация отладки 
            }
        }

        public bool IsPossible => CheckState("H") && CheckState("F0F1");
    }

    public RobHand[] Inversion1(Coord C, int[] Index)
    {
        var z = c1x + f1x;
        var XY1 = new Coord([Index[0] * Ctx.step_x, Index[1] * Ctx.step_y]);
        if(XY1.Y < C.Y)
        {
            var D = C - XY1;
            var beta = Math.Acos(D.X / D.Norm);
            var alpha = Math.Asin(z / D.Norm);
            if(alpha is double.NaN || beta is double.NaN) return null;
            else
            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, -c1y, Aff(e1, -c1x, new Coord([C.X, C.Y])));
                var l1 = D * e0 - c1y;                         
                var h = new RobHand(){HandLine = [e0, q0], HandPos = XY1, HandLen = -l1};
                var qh = Aff(e1, hx, EvalLine([e0, q0], -l1 - hy));
                var hh = new RobHand(){HandLine = [e0, q0], HandPos = qh, HandLen = -l1 - hy};
                return [h, hh];
            }
        }   
        else
        {
            var D = XY1 - C;
            var beta = Math.Acos(D.X / D.Norm);
            var alpha = Math.Asin(z / D.Norm);
            if(alpha is double.NaN || beta is double.NaN) return null;
            else
            {
                var phi = alpha + beta;
                var e0 = Ein(phi);
                var e1 = Compl(e0);
                var q0 = Aff(e0, c1y, Aff(e1, c1x, new Coord([C.X, C.Y])));
                var l1 = D * e0 - c1y;                         
                var h = new RobHand(){HandLine = [e0, q0], HandPos = XY1, HandLen = l1};
                var qh = Aff(e1, hx, EvalLine([e0, q0], l1 + hy));
                var hh = new RobHand(){HandLine = [e0, q0], HandPos = qh, HandLen = l1 + hy};
                return [h, hh];
            }
        }
    }

    public RobState Inversion02(Coord xy2, Coord xy0, int[] xy1)
    {
        if(xy2.Y < xy0.Y)
        {  
            var D = xy0 - xy2;
            var theta = Math.Atan2(D.Y, D.X);
            var alpha = Math.Asin((f0x - f2x) / D.Norm);
            if(theta is double.NaN) return null;
            else
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * D - f2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, f2x, xy2);
                var c =  Aff(e1, c2x, Aff(e0, -c2y, q2));
                var q0 = Aff(e0, f2y, q2);
                var h0 = new RobHand{HandLine = [e0, q0], HandPos = xy0, HandLen = l0};
                var H1_H = Inversion1(c, xy1);
                var State = new RobState(){F2 = xy2, C = c, Hands = new RobHand[3]};
                State.SetRobHand(h0, "h0");
                if(H1_H is not null)
                {
                    State.SetRobHand(H1_H[0], "h1");
                    State.SetRobHand(H1_H[1], "h");
                    return State;
                }
                return null;
            }
        }
        else
        {
            var D = xy2 - xy0;
            var theta = Math.Atan2(D.Y, D.X);
            var alpha = Math.Sin((f0x - f2x) / D.Norm);
            if(theta is double.NaN) return null;
            else
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * D - f2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, -f2x, xy2);
                var c =  Aff(e1, -c2x, Aff(e0, -c2y, q2));
                var q0 = Aff(e0, -f2y, q2);
                var h0 = new RobHand{HandLine = [e0, q0], HandPos = xy0, HandLen = -l0};
                var H1_H = Inversion1(c, xy1);
                var State = new RobState{F2 = xy2, C = c, Hands = new RobHand[3]};
                State.SetRobHand(h0, "h0");
    
                if(H1_H is not null)
                {
                    State.SetRobHand(H1_H[0], "h1");
                    State.SetRobHand(H1_H[1], "h");
                    return State;
                }
                return null;
            }
        }
    }

    public RobState Inversion(int[] F2, int[] F0, int[] F1)
    {
        Coord XY2 = new([F2[0] * Ctx.step_x, F2[1] * Ctx.step_y]), XY0 = new([F0[0] * Ctx.step_x, F0[1] * Ctx.step_y]);
        return Inversion02(XY2, XY0, F1);
    }

    private int[] DefaultF0, DefaultF2Upper, DefaultF2Lower, DefaultH;

    public int[] GetF0 => DefaultF0;

    public int[] GetF2Upper => DefaultF2Upper;

    public int[] GetF2Lower => DefaultF2Lower;

    public List<int[]> HIndexesUpper, HIndexesLower;

    public List<int[]> F0IndexesUpper, F0IndexesLower;

    public List<int[]> GetIndexes(int[] Point, string Param)
    {
        List<int[]> IndexesArray = [];
        switch(Param)
        {
            case "HUpper":
                IndexesArray = HIndexesUpper;
                break;
            case "HLower":
                IndexesArray = HIndexesLower;
                break;
            case "F0Upper":
                IndexesArray = F0IndexesUpper;
                break;
            case "F0Lower":
                IndexesArray = F0IndexesLower;
                break;
            default: throw new Exception("НЕИЗВЕСТНОЕ ЗНАЧЕНИЕ ПАРАМЕТРА ВЫБОРА");
        }
        var Res = new List<int[]>();
        IndexesArray.ForEach(I => Res.Add([I[0] - DefaultF0[0] + Point[0], I[1] - DefaultF0[1] + Point[1]]));
        return Res.FindAll(I => {var Name = $"{I[0]}_{I[1]}"; return Ctx.map.Keys.ToList().Contains(Name) && Ctx.map[Name].state != "Inaccessible";});
    }

    private DataContext TestCtx;

    public DataContext GetTestCtx => TestCtx;

    private bool CheckUpperH(int[] H)
    {
        var S = Inversion(DefaultF2Upper, DefaultF0, H);
        if(S is not null && S.IsPossible) return S.AngleIsPossible1;
        return false;
    }

    private bool CheckLowerH(int[] H)
    {
        var S = Inversion(DefaultF2Lower, DefaultF0, H);
        if(S is not null && S.IsPossible) return H[1] < DefaultF2Lower[1] && S.AngleIsPossible;
        return false;
    }

    private bool CheckUpperF0(int[] F0)
    {
        var S = Inversion([F0[0] - 1, F0[1] + 5], F0, DefaultH);
        if(S is not null && S.IsPossible) return S.AngleIsPossible1;
        return false;
    }

    private bool CheckLowerF0(int[] F0)
    {
         var S = Inversion([F0[0] + 1, F0[1] - 5], F0, DefaultH);
        if(S is not null && S.IsPossible) return DefaultH[1] < F0[1] - 5 && S.AngleIsPossible;
        return false;
    }

    public RobotModel(DataContext SomeCtx, int[] StartPos)
    {
        Ctx = SomeCtx;
        DefaultF0 = StartPos;
        (DefaultF2Upper, DefaultF2Lower) = ([DefaultF0[0] - 1, DefaultF0[1] + 5], [DefaultF0[0] + 1, DefaultF0[1] - 5]);
        DefaultH = StartPos;
        TestCtx = new DataContextForTestPipeDesk(Ctx, 50);
        HIndexesUpper = TestCtx.map.Values.ToList().FindAll(Pipe => CheckUpperH(Pipe.index)).ConvertAll(Pipe => Pipe.index);
        HIndexesLower = TestCtx.map.Values.ToList().FindAll(Pipe => CheckLowerH(Pipe.index)).ConvertAll(Pipe => Pipe.index);
        var F0IndexesLowerBuffer = TestCtx.map.Values.ToList().FindAll(Pipe => CheckLowerF0(Pipe.index)).ConvertAll(Pipe => Pipe.index);
        var F0IndexesUpperBuffer = TestCtx.map.Values.ToList().FindAll(Pipe => CheckUpperF0(Pipe.index)).ConvertAll(Pipe => Pipe.index);
        (F0IndexesLower, F0IndexesUpper) = ([], []);
        F0IndexesLowerBuffer.ForEach(I => F0IndexesLower.Add([I[0], I[1]]));
        F0IndexesUpperBuffer.ForEach(I => F0IndexesUpper.Add([I[0], I[1]]));
        //Console.WriteLine(HIndexesLower.Count);
        //Console.WriteLine(HIndexesUpper.Count);
    }
}