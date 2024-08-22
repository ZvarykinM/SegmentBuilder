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

class RobotModel : UnorientedRobotModel
{

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