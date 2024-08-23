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
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.Z3;
using System.Formats.Tar;

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

class RobHand: ICloneable
{
    public Line HandLine;

    public Coord HandPos;

    public Coord HandLineLikeVector => HandLine[0] - HandLine[1];

    public double HandLen;

    public override string ToString() => $"HandLine0 = {HandLine[0]}; HandLine1 = {HandLine[1]}; HandPos = {HandPos}; HandLen = {Math.Round(HandLen, 3)}";

    public static double dist2(Coord q0, Coord q1) => (q0 - q1).Norm;
    
    public static Coord EvalLine(Line l, double t) => new([l[1].X + t * l[0].X, l[1].Y + t * l[0].Y]);
    
    public double AbsDistance => dist2(HandLine[1], EvalLine(HandLine, HandLen));

    public object Clone() => new RobHand(){HandLine = [HandLine[0], HandLine[1]], HandPos = new([HandPos.X, HandPos.Y]), HandLen = HandLen};

    public RobHand Copy => Clone() as RobHand;
}

class RobState: ICloneable
{
    public const double rad = 76.0, hx = 31.0, hy = 44.0,
    f1a = 0.06, f0x = 41.0, f1x = 41.0, c1x = 15.0, c1y = 74.5,
    f2x = 25.0, f2y = 27.8, c2x = 18.0, c2y = 46.0, l1_min = 67.0,
    l1_max = 183.8, l0_min = 62.3, l0_max = 269.6, phi_step = 0.0001,
    l_step = 0.04, flat_width = 792, flat_height = 540;
    
    public Coord F2, C;

    public RobHand[] Hands = new RobHand[3];

    public override string ToString() => $"F2 = {F2};\n C = {C};\n H0 = {Hands[0]};\n H1 = {Hands[1]};\n HH = {Hands[2]}";

    public void SetRobHand(RobHand H, string HandIndex)
    {
        var i = HandIndex switch
        {
            "h0" => 0,
            "h1" => 1,
            "h" => 2,
            _ => throw new Exception("НЕИЗВЕСТНЫЙ ИНДЕКС ПАЛЬЦА")
        };
        Hands[i] = H.Copy;
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

    public static double AngNorm(double x) => x < 0.0 ? x + 2 * Math.PI : x;

    public bool CheckState(string arg)
    {
        if(this is null) return false;
        switch(arg)
        {
            case "H":
                var lh_max = l1_max + hy;
                var lh_min = l1_min + hy;
                return Hands[2].AbsDistance < lh_max && Hands[2].AbsDistance > lh_min;
            case "F1":
                return Hands[1].AbsDistance < l1_max && Hands[1].AbsDistance > l1_min;
            case "F0F1":
                var b1 = (Hands[1].AbsDistance < l1_max) && (Hands[1].AbsDistance > l1_min);
                var b0 = (Hands[0].AbsDistance < l0_max) && (Hands[0].AbsDistance > l0_min);
                return b0 && b1;
            default: throw new Exception("НЕИЗЕВЕСТНЫЙ ПАРАМЕТР ПРОВЕРКИ"); //здесь может потребоваться дополнительная информация отладки 
        }
    }

    public bool IsPossible => CheckState("H") && CheckState("F0F1");
}

class UnorientedRobotModel
{
    public static Coord Compl(Coord l) => new([l.Y, -l.X]);

    public static Coord Ein(double phi) => new([Math.Cos(phi), Math.Sin(phi)]);

    public static Coord Aff(Coord e, double t, Coord q) => new([q.X + t * e.X, q.Y + t * e.Y]);

    
    public RobHand[] Inversion1(Coord C, int[] Index)
    {
        var z = RobState.c1x + RobState.f1x;
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
                var q0 = Aff(e0, -RobState.c1y, Aff(e1, -RobState.c1x, new Coord([C.X, C.Y])));
                var l1 = D * e0 - RobState.c1y;                         
                var h = new RobHand(){HandLine = [e0, q0], HandPos = XY1, HandLen = -l1};
                var qh = Aff(e1, RobState.hx, RobHand.EvalLine([e0, q0], -l1 - RobState.hy));
                var hh = new RobHand(){HandLine = [e0, q0], HandPos = qh, HandLen = -l1 - RobState.hy};
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
                var q0 = Aff(e0, RobState.c1y, Aff(e1, RobState.c1x, new Coord([C.X, C.Y])));
                var l1 = D * e0 - RobState.c1y;                         
                var h = new RobHand(){HandLine = [e0, q0], HandPos = XY1, HandLen = l1};
                var qh = Aff(e1, RobState.hx, RobHand.EvalLine([e0, q0], l1 + RobState.hy));
                var hh = new RobHand(){HandLine = [e0, q0], HandPos = qh, HandLen = l1 + RobState.hy};
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
            var alpha = Math.Asin((RobState.f0x - RobState.f2x) / D.Norm);
            if(theta is double.NaN) return null;
            else
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * D - RobState.f2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, RobState.f2x, xy2);
                var c =  Aff(e1, RobState.c2x, Aff(e0, -RobState.c2y, q2));
                var q0 = Aff(e0, RobState.f2y, q2);
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
            var alpha = Math.Sin((RobState.f0x - RobState.f2x) / D.Norm);
            if(theta is double.NaN) return null;
            else
            {
                var e0 = Ein(theta - alpha);
                var l0 = e0 * D - RobState.f2y;
                var e1 = Compl(e0);
                var q2 = Aff(e1, -RobState.f2x, xy2);
                var c =  Aff(e1, -RobState.c2x, Aff(e0, -RobState.c2y, q2));
                var q0 = Aff(e0, -RobState.f2y, q2);
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

    public DataContext Ctx;
    
    public List<int[]> HoseArray = [], PathArray = []; //HoseArray - для хранения H-переменных, сопряжённых с заданной; PathArray - для хранения F-переменных

    public int[] F0, F2_0, F2_1, H;

    public DataContext TestData;

    public bool CheckF0(int[] Index)
    {
        int[] F2_Upper = [Index[0] - 1, Index[1] + 5], F2_Lower = [Index[0] + 1, Index[1] - 5];
        RobState SUpper = Inversion(F2_Upper, Index, H), SLower = Inversion(F2_Lower, Index, H);
        return (SUpper is not null && SUpper.IsPossible && SUpper.AngleIsPossible1) || (SLower is not null && SLower.IsPossible && SLower.AngleIsPossible);
    }

    public bool CheckH(int[] Index)
    {
        RobState SUpper = Inversion(F2_0, F0, Index), SLower = Inversion(F2_1, F0, Index);
        return (SUpper is not null && SUpper.IsPossible && SUpper.AngleIsPossible1) || (SLower is not null && SLower.IsPossible && SLower.AngleIsPossible);
    }

    public void Calculate(DataContext SomeCtx, int[] StartPos)
    {
        (Ctx, F0, F2_0, F2_1, H) = (SomeCtx, StartPos, [StartPos[0] - 1, StartPos[1] + 5], [StartPos[0] + 1, StartPos[1] - 5], StartPos);
        TestData = new DataContextForTestPipeDesk(Ctx, 50);
        TestData.map.Values.ToList().ConvertAll(V => V.index).FindAll(CheckH).ForEach(I => HoseArray.Add([I[0] - F0[0], I[1] - F0[1]]));
        TestData.map.Values.ToList().ConvertAll(V => V.index).FindAll(CheckF0).ForEach(I => PathArray.Add([I[0] - F0[0], I[1] - F0[1]]));
    }

    public List<int[]> FindPositionsForCurrIndex(int[] Index, string Param)
    {
        var Res = new List<int[]>();
        List<int[]> Positions = Param switch
        {
            "Hose" => HoseArray,
            "Path" => PathArray,
            _ => throw new Exception("НЕИЗВЕСТНЫЙ ПАРАМЕТР ВЫБОРА"),
        };
        Positions.ForEach(I => Res.Add([I[0] + Index[0], I[1] + Index[1]]));
        return Res.Select(I => $"{I[0]}_{I[1]}").Where(S => Ctx.map.ContainsKey(S) && Ctx.map[S].state != "Inaccessible").Select(S => S.Split('_')
                  .Select(n => Convert.ToInt32(n)).ToArray()).ToList();
    }
}

class IndexCover
{
    private readonly Context Ctx;

    private readonly UnorientedRobotModel Robot;

    private int[] Index;

    public IndexCover(UnorientedRobotModel SomeRobot, Context Logic, int[] SomeIndexPair) => (Ctx, Robot, IndexPair) = (Logic, SomeRobot, SomeIndexPair);

    private List<int[]> ConjugateIndexes(int[] SomeIndex, string arg) => Robot.FindPositionsForCurrIndex(SomeIndex, arg);

    private List<BoolExpr> ConjugateHVars = [], ConjugateFVars = [];

    public int[] IndexPair
    {
        get => Index;
        set
        {
            Index = value;
            ConjugateHVars = ConjugateIndexes(Index, "Hose").ConvertAll(I => Ctx.MkBoolConst($"H{I[0]}_{I[1]}"));
            ConjugateFVars = ConjugateIndexes(Index, "Path").ConvertAll(I => Ctx.MkBoolConst($"F{I[0]}_{I[1]}"));
        }
    }

    public string Name => $"{Index[0]}_{Index[1]}";

    public BoolExpr FExpr => Ctx.MkBoolConst("F" + Name);

    public BoolExpr HExpr => Ctx.MkBoolConst("H" + Name);

    public BoolExpr PathConstr => Ctx.MkImplies(HExpr, Ctx.MkAtLeast(ConjugateFVars, 1));

    public BoolExpr HoseConstr => Ctx.MkImplies(FExpr, Ctx.MkAnd(ConjugateHVars));

    private bool HValue, FValue;

    public void SetValues(Solver Calculator) => 
        (HValue, FValue) = (Calculator.Model.Evaluate(HExpr).ToString() == "true", Calculator.Model.Evaluate(FExpr).ToString() == "true");

    public bool GetHValue => HValue;

    public bool GetFValue => FValue;
}

class PathFinder
{
    private readonly Context Ctx;

    private readonly DataContext DataCtx;

    public List<IndexCover> Indexes;

    public PathFinder(DataContext SomeDataContext)
    {
        DataCtx = SomeDataContext;
        using(Ctx = new())
        {
            var Robot = new UnorientedRobotModel();
            Robot.Calculate(DataCtx, [19, 17]);
            Indexes = DataCtx.map.Values.ToList().ConvertAll(V => new IndexCover(Robot, Ctx, V.index));
            var CommonPathConstr = Ctx.MkAnd(Indexes.ConvertAll(I => I.PathConstr));
            var CommonHoseConstr = Ctx.MkAnd(Indexes.ConvertAll(I => I.HoseConstr));
            var Goal = Ctx.MkAnd(Indexes.ConvertAll(I => I.HExpr));
            var Calc = Ctx.MkSolver();
            Calc.Add(Ctx.MkAnd([CommonPathConstr, CommonHoseConstr, Goal]));
            Indexes.ForEach(I => I.SetValues(Calc));
        }
    }
}