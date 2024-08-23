using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using ScottPlot;
using ScottPlot.Avalonia;
using KVP = System.Collections.Generic.KeyValuePair<string, ScottPlot.Color>;
using HolePair = System.Collections.Generic.KeyValuePair<string, DiscreteRobotImplementation.HoleState>;
using Avalonia.Media;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace DiscreteRobotImplementation;

class HoleState
{
    public string state{get; set;}

    public string saved{get; set;}

    public int[] index{get; set;}

    public double[] geom_coord{get;set;}

    public override string ToString() => $"\"state\":{state},\"saved\":{saved},\"index\":[{index[0]},{index[1]}],\"geom_coord\"[{geom_coord[0]},{geom_coord[1]}]";
}

class FingerState
{
    public bool valve{get; set;} //???

    public double[] position{get; set;}
}

class DataContext
{
    public double step_x{get; set;}

    public double step_y{get; set;}

    public double radius{get; set;}

    public string name{get; set;}

    public Dictionary<string, HoleState> map{get; set;}

    public Dictionary<string, FingerState> fingers{get; set;}    
}

class DataContextForTestPipeDesk: DataContext
{
    private int Size;

    private List<int[]> MakeRow(int RowNum)
    {
        var Res = new List<int[]>();
        for(var i = 0; i < Size; i++)
            Res.Add([i, RowNum]);
        return Res;
    }

    private List<int[]> MakeOddOrNonOddRow(List<int[]> RowElements, bool IsOdd = true) => RowElements.Select((Elem, Index) => new{Index, IndexPair = Elem}).Where(IndexObj => IsOdd ? IndexObj.Index % 2 == 0 : IndexObj.Index % 2 != 0).ToList().ConvertAll(Obj => Obj.IndexPair);

    private List<int[]> MakeOddRow(List<int[]> SomeRow) => MakeOddOrNonOddRow(SomeRow);

    private List<int[]> MakeNonOddRow(List<int[]> SomeRow) => MakeOddOrNonOddRow(RowElements: SomeRow, IsOdd: false);

    public DataContextForTestPipeDesk(DataContext Ctx, int N)
    {
        (step_x, step_y, radius, name, Size) = (Ctx.step_x, Ctx.step_y, Ctx.radius, $"({N} x {N})-PipeDesk", N);
        var Rows = new List<List<int[]>>();
        for(var i = 0; i < N; i++)
        {
            var Row = MakeRow(i);
            if(i % 2 == 0)
                Rows.Add(MakeOddRow(Row));
            else Rows.Add(MakeNonOddRow(Row)); 
        }
        map = new();
        Rows.ForEach(Row => Row.ConvertAll(IndexPair => CreateHoleState(IndexPair)).ForEach(HS => map.Add(HS.Key, HS.Value)));
    }

    private HolePair CreateHoleState(int[] Index) => new($"{Index[0]}_{Index[1]}", new(){state = "Neutral", saved = "Neutral", index = Index, geom_coord = [Index[0] * step_x, Index[1] * step_y]});

    public string CreateDataContextJson => JsonSerializer.Serialize<DataContext>(CreateDataContextExample);

    public DataContext CreateDataContextExample => new(){step_x = this.step_x, step_y = this.step_y, radius = this.radius, name = this.name, map = this.map};
}