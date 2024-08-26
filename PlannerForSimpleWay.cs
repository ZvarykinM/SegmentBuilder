
using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using Microsoft.Z3;
using DiscreteRobotImplementation;
using KVP = System.Collections.Generic.KeyValuePair<string, ScottPlot.Color>;
using System.Linq;
namespace SegmentBuilder;

class GridModel
{
    private Dictionary<string, ScottPlot.Color> HoleStatesDescription = new(new List<KVP>([new KVP("Done", ScottPlot.Colors.Green),
                                                                                 new KVP("Finger", ScottPlot.Colors.Brown),
                                                                                 new KVP("Planned", ScottPlot.Colors.Yellow),
                                                                                 new KVP("Goal", ScottPlot.Colors.Orange),
                                                                                 new KVP("Inaccessible", ScottPlot.Colors.Red),
                                                                                 new KVP("Accessible", ScottPlot.Colors.Blue),
                                                                                 new KVP("Neutral", ScottPlot.Colors.LightBlue)]));
    public ScottPlot.Color GetColorOfState(HoleState SomeHoleState) => HoleStatesDescription[SomeHoleState.state];

    private DataContext DataCtx;

    private List<int[]> AllIndexCoords;

    private string PathToFileWithGridContext;

    public string PipeDeskName
    {
        private set => PathToFileWithGridContext = value;

        get => DataCtx.name;
    }

    public List<int[]> GetAllIndexCoords => AllIndexCoords;

    public double XStep => DataCtx.step_x;

    public double YStep => DataCtx.step_y;

    public double Radius => DataCtx.radius;

    public DataContext GetDataContext => DataCtx;

    public string this[string HoleName]
    {
        set => DataCtx.map[HoleName].state = value;

        get => $"state of {HoleName} = {DataCtx.map[HoleName].state}";
    }

    public void SetStateByStringKey(string HoleName, string State)
    {
        var ChangableState = DataCtx.map[HoleName].state;
        if(ChangableState != "Inaccessible") 
            DataCtx.map[HoleName].state = State;
    }

    public static string IndexCoordToString(int[] Index) => $"{Index[0]}_{Index[1]}";

    public Coord this[int[] Index] => new(DataCtx.map[IndexCoordToString(Index)].geom_coord); //получить геометрическую координату трубки по индексу

    public GridModel(string PathToFile)
    {
        PathToFileWithGridContext = PathToFile;
        using(var sr = new StreamReader(PathToFile))
        {
            PathToFile = sr.ReadLine();
        } 
        DataCtx = JsonSerializer.Deserialize<DataContext>(PathToFile);
        AllIndexCoords = new();
        foreach(var hole in DataCtx.map)
            if(hole.Value.state != "Inaccessible") AllIndexCoords.Add(hole.Value.index);
    }

    public GridModel(DataContext SomeDataContext)
    {
        DataCtx = SomeDataContext;
        AllIndexCoords = new();
        foreach(var hole in DataCtx.map)
            AllIndexCoords.Add(hole.Value.index);
    }

    public GridModel Copy => new(DataCtx);
}

class SecondPlanner: IDisposable
{
    public class DiscreteIndex(int[] IntPair, GridModel SomeGridModel, Context SomeContext): IEquatable<DiscreteIndex>
    {
        protected int[] Index = IntPair;

        public int[] GetIndex => Index;

        public GridModel GM = SomeGridModel;

        public Context Ctx = SomeContext;

        private const int XStart = 19, YStart = 17;

        public static UnorientedRobotModel Initialize(DataContext SomeCtx)
        {
            var Robot = new UnorientedRobotModel();
            Robot.Calculate(SomeCtx, [XStart, YStart]);
            return Robot;
        }

        public UnorientedRobotModel Robot = Initialize(SomeGridModel.GetDataContext);

        public bool IsKeyPosition => GM.GetDataContext.map[$"{Index[0]}_{Index[1]}"].state == "Finger";

        public bool Equals(DiscreteIndex SomeDiscreteIndex) => Index[0] == SomeDiscreteIndex.Index[0] && Index[1] == SomeDiscreteIndex.Index[1];

        public List<DiscreteIndex> MakeHexagoneIndexes(string Param)
        {
            var PVars = Robot.FindPositionsForCurrIndex(Index, Param);
            return PVars.Where(Ind => GM.GetDataContext.map.Keys.
                            Contains($"{Ind[0]}_{Ind[1]}") && GM.GetDataContext.map[$"{Ind[0]}_{Ind[1]}"].state != "Inaccessible")
                            .ToList().ConvertAll(Index => new DiscreteIndex(Index, GM, Ctx));
        }

        public List<string> MakeHexagone(string Param)
        {
            var PVars = Robot.FindPositionsForCurrIndex(Index, Param).ConvertAll(IndexPair => $"{IndexPair[0]}_{IndexPair[1]}");
            return PVars.Where(Ind => GM.GetDataContext.map.Keys.Contains(Ind) && GM.GetDataContext.map[Ind].state != "Inaccessible").ToList();
        }

        private BoolExpr GetPBoolExpr(string Index) => Ctx.MkBoolConst($"P_{Index}");

        private BoolExpr GetCBoolExpr(string Index) => Ctx.MkBoolConst($"C_{Index}");

        public List<BoolExpr> GetHexagoneFor_P => MakeHexagone("Path").ConvertAll(GetPBoolExpr);

        public List<BoolExpr> GetHexagoneFor_C => MakeHexagone("Hose").ConvertAll(GetCBoolExpr);

        public BoolExpr GetBoolExpr_P => Ctx.MkBoolConst($"P_{Index[0]}_{Index[1]}");

        public BoolExpr GetBoolExpr_C => Ctx.MkBoolConst($"C_{Index[0]}_{Index[1]}");

        //проверка соединённости одним ребром со всеми рядом стоящими точками
        public bool HasNearestPoints => NearestPoints.Count != 0;
        
        public List<string> NearestPoints
        {
            get
            {
                var DataCtx = GM.GetDataContext.map;
                return DataCtx.Keys.ToList().FindAll(PipePos => Math.Abs(Index[0] - DataCtx[PipePos].index[0]) <= 1 && Math.Abs(Index[1] - DataCtx[PipePos].index[1]) <= 1);
            }
        }

        public List<BoolExpr> GetConclusionsForNearestPoints => NearestPoints.ConvertAll(C => Ctx.MkEq(Ctx.MkBoolConst($"E_{Index[0]}_{Index[1]}_{C}"), Ctx.MkBool(true)));
    }
                        
    private uint M; //параметр оптимизации

    private Context PathCtx;


    private Context Ctx;
    
    private GridModel GM;

    public GridModel GetGridModel => GM;

    private string OutputPath = "DebugOutput.txt";

    private List<int[]> AllOneFingeredStates;

    private string PathToOutputFile;

    public SecondPlanner(GridModel SomeGridModel, string _PathToOutputFile = "Output.txt")
    {
        GM = SomeGridModel;
        PathToOutputFile = _PathToOutputFile;
        AllOneFingeredStates = GM.GetAllIndexCoords;
    }

    public static string PNameToDataContextName(string PName) => PName.Remove(0, 2);

    //C_i_j -> AtLeast({P_i_j_{-ko},...,P_i_j+{ko}}, 1) (1)
    //P_i_j -> P_i_j_{-ko} & ... & P_i_j_{+ko}          (2)
    //AtMost({P_i_j}, M) = true                         (3)
    //C_i_j_0 & ... & C_i_j_N                           (4)
    //Final Formula: (1) & (2) & (3) & (4)

    private List<DiscreteIndex> IndexesLikeBoolVars;

    public Dictionary<int[], List<int[]>> HoseIndexesPhiFunc = [];

    private BoolExpr Source(DiscreteIndex IX) => Ctx.MkImplies(IX.GetBoolExpr_C, Ctx.MkAtLeast(IX.GetHexagoneFor_P, 1));

    private BoolExpr Cover(DiscreteIndex IX) => Ctx.MkImplies(IX.GetBoolExpr_P, Ctx.MkAnd(IX.GetHexagoneFor_C));

    private string SMTSolve(uint M = 2)
    {
        using(Ctx = new Context())
        {
            Console.WriteLine("Process started");
            //создаём наборы переменных состояния
            IndexesLikeBoolVars = AllOneFingeredStates.ConvertAll(IndexPair => new DiscreteIndex(IntPair: IndexPair, SomeGridModel: GM, SomeContext: Ctx));
            var Solver = Ctx.MkSolver();
            //добавляем все необходимые условия
            var cs = new List<BoolExpr>();
            var ps = new List<BoolExpr>();
            var g = Ctx.MkAnd(Ctx.MkAnd(Ctx.MkAnd(IndexesLikeBoolVars.ConvertAll(IX => IX.GetBoolExpr_C)), Ctx.MkBool(true)));
            //var g = Ctx.MkAnd(IndexesLikeBoolVars.ConvertAll(IX => IX.GetBoolExpr_C));
            var ck = Ctx.MkAtMost(IndexesLikeBoolVars.ConvertAll(IX => IX.GetBoolExpr_P), M);
            IndexesLikeBoolVars.ForEach(IX => {cs.Add(Source(IX)); ps.Add(Cover(IX));});
            Solver.Add(Ctx.MkAnd([Ctx.MkAnd(cs), Ctx.MkAnd(ps), g, ck])) ;//Ctx.MkEq(g, Ctx.MkBool(true)), ck])); ПРОВЕРИТЬ g на true - опционально
            Console.WriteLine($"{Solver.Check()} for {GM.PipeDeskName}");
            Console.WriteLine(Solver.Model is not null ? "Model exists" : "Model doesn't exist");
            foreach(var IX in IndexesLikeBoolVars)
            {
                var PName = IX.GetBoolExpr_P.FuncDecl.Name.ToString().Remove(0, 2);
                var PValue = Solver.Model.Evaluate(IX.GetBoolExpr_P).ToString() == "true";
                var CName = IX.GetBoolExpr_C.FuncDecl.Name.ToString().Remove(0, 2);
                var CValue = Solver.Model.Evaluate(IX.GetBoolExpr_C).ToString() == "true";
                if(CValue) GM.SetStateByStringKey(CName, "Done");
                if(PValue)
                {
                    GM.SetStateByStringKey(CName, "Finger");
                    HoseIndexesPhiFunc.Add(IX.GetIndex, IX.MakeHexagoneIndexes("Hose").ConvertAll(I => I.GetIndex));
                }
            }
            Console.WriteLine("Process ended");
            using(var SW = new StreamWriter(PathToOutputFile))
            {
                SW.Write(JsonSerializer.Serialize<DataContext>(GM.GetDataContext));
            }
            return Solver.Check().ToString();
        }
    }

    public string Solve(uint M = 0)
    {
        var D1 = DateTime.Now;
        var Res = SMTSolve(M);
        var D2 = DateTime.Now;
        Console.WriteLine($"Время исполнения процесса:  {(D2 - D1).Milliseconds} миллисекунд");
        return Res;
    }

    public void Dispose(){}
}