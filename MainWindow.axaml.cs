using Avalonia.Controls;
using DiscreteRobotImplementation;
using System.IO;
using System.Text.Json;
using ScottPlot.Avalonia;
using System.Collections.Generic;
using KVP = System.Collections.Generic.KeyValuePair<string, ScottPlot.Color>;
using System;
using Avalonia.Interactivity;
using System.Linq;
using Avalonia.Input;
using System.Timers;

namespace SegmentBuilder;

public partial class MainWindow : Window
{
    class SchemaRenderer(AvaPlot GridPlot, DataContext SomeDataCtx, RobotModel SomeRobot)
    {
        private Dictionary<string, ScottPlot.Color> HoleStatesDescription = new(new List<KVP>([new KVP("Done", ScottPlot.Colors.DarkGreen),
                                                                                            new KVP("Finger", ScottPlot.Colors.Brown),
                                                                                            new KVP("Planned", ScottPlot.Colors.Yellow),
                                                                                            new KVP("Goal", ScottPlot.Colors.Orange),
                                                                                            new KVP("Inaccessible", ScottPlot.Colors.Red),
                                                                                            new KVP("Accessible", ScottPlot.Colors.Blue),
                                                                                            new KVP("Neutral", ScottPlot.Colors.LightBlue)]));
        public AvaPlot Grid = GridPlot;

        public DataContext Ctx = SomeDataCtx;

        public bool IsForTest => Ctx is DataContextForTestPipeDesk;

        private RobotModel Robot = SomeRobot;

        private Dictionary<string, ScottPlot.Plottables.Ellipse> PipeSchemas = [];

        private Dictionary<string, ScottPlot.Plottables.Text> Labels = [];

        private void DrawPipe(string Name, HoleState PipeDescription)
        {
            var C = Grid.Plot.Add.Circle(PipeDescription.geom_coord[0], PipeDescription.geom_coord[1], Ctx.radius);
            C.FillColor = HoleStatesDescription[PipeDescription.state];
            C.LineColor = ScottPlot.Colors.Black;
            var Text = Grid.Plot.Add.Text(Name, PipeDescription.geom_coord[0], PipeDescription.geom_coord[1]);
            Text.LabelFontSize = 9;
            try{PipeSchemas.Add(Name, C);}
            catch{PipeSchemas[Name] = C;}
            try{Labels.Add(Name, Text);}
            catch{Labels[Name] = Text;}
            Grid.Refresh();
        }

        public void DrawPipeDesk()
        {
            foreach(var p in  Ctx.map)
                DrawPipe(p.Key, p.Value);
            Grid.Refresh();
        }

        private List<ScottPlot.Plottables.Ellipse> HSegment = [];

        private List<ScottPlot.Plottables.Ellipse> F0Segment = [];

        private ScottPlot.Plottables.Ellipse IndexToCircle(int[] Index, string Param)
        {
            var C = Grid.Plot.Add.Circle(Index[0] * Ctx.step_x, Index[1] * Ctx.step_y, Ctx.radius);
            C.FillColor = Param switch
            {
                "HU" => ScottPlot.Colors.OrangeRed,
                "HL" => ScottPlot.Colors.OliveDrab,
                "F0" => ScottPlot.Colors.DarkBlue,
                "F2" => ScottPlot.Colors.DarkRed,
                _ => throw new Exception()
            };
            return C;
        }

        public void ClearAll()
        {
            new List<List<ScottPlot.Plottables.Ellipse>>([PipeSchemas.Values.ToList(), HSegment, F0Segment]).ForEach(L => L.ForEach(C => Grid.Plot.Remove(C)));
            Labels.Values.ToList().ForEach(Text => Grid.Plot.Remove(Text));
            Grid.Refresh();
        }

        public void ClearPartial() => new List<List<ScottPlot.Plottables.Ellipse>>([HSegment, F0Segment]).ForEach(L => L.ForEach(C => Grid.Plot.Remove(C)));
        
        public void DrawSegment(int[] PipeIndex = null)
        {
            //ClearAll();
            var MoveStep = PipeIndex ?? Robot.GetF0;
            Robot.HIndexesUpper.ForEach(I => HSegment.Add(IndexToCircle(I, "HU")));
            Robot.HIndexesLower.ForEach(I => HSegment.Add(IndexToCircle(I, "HL")));
            HSegment.Add(IndexToCircle(Robot.GetF0, "F0"));
            HSegment.Add(IndexToCircle(Robot.GetF2Lower, "F2"));
            HSegment.Add(IndexToCircle(Robot.GetF2Upper, "F2"));
            Grid.Refresh();
        }

        public void DrawPathConstraints()
        {
            Robot.F0IndexesLower.ForEach(I => F0Segment.Add(IndexToCircle(I, "F0")));
            Robot.F0IndexesUpper.ForEach(I => F0Segment.Add(IndexToCircle(I, "F0")));
            F0Segment.Add(IndexToCircle(Robot.GetF0, "H"));
            Grid.Refresh();
        }

        private List<ScottPlot.Plottables.Ellipse> ForCurrElementaryDiscreteIndex = [];

        public void DrawElementaryDiscreteIndex(ElementaryDiscreteIndex Index, string Param)
        {
            List<int[]> ConjugateIndexes = [];
            string flag1 = "", flag2 = "";
            (ConjugateIndexes, flag1, flag2) = Param switch
            {
                "Hose" => (Robot.GetIndexes(Index.GetIndexValue, "HLower"), "F0", "HL"),
                "Path" => (Robot.GetIndexes(Index.GetIndexValue, "F0Upper"), "HU", "F0"),
                _ => throw new Exception("НЕИЗВЕСТНОЕ ЗНАЧЕНИЕ ПАРАМЕТРА ВЫБОРА"),
            };
            ConjugateIndexes.ForEach(I => ForCurrElementaryDiscreteIndex.Add(IndexToCircle(I, flag2)));
            ForCurrElementaryDiscreteIndex.Add(IndexToCircle(Robot.GetF0, flag1));
            Grid.Refresh();
        }

        private List<ScottPlot.Plottables.Ellipse> CommonHoseSchema = [], CommonPathSchema = [];

        public void DrawCommonHoseSchema(List<int[]> HosePipes, int[] Index)
        {
            CommonHoseSchema.ForEach(Grid.Plot.Remove);
            CommonHoseSchema.Clear();
            HosePipes.ForEach(I => CommonHoseSchema.Add(IndexToCircle(I, "HL")));
            CommonHoseSchema.Add(IndexToCircle(Index, "F0"));
            Grid.Refresh();
        }

        public void DrawCommonPathSchema(List<int[]> PathPipes, int[] Index)
        {
            CommonPathSchema.ForEach(Grid.Plot.Remove);
            CommonPathSchema.Clear();
            PathPipes.ForEach(I => CommonPathSchema.Add(IndexToCircle(I, "F0")));
            CommonPathSchema.Add(IndexToCircle(Index, "HL"));
            Grid.Refresh();
        }

        public void ClearCommon(string arg)
        {
            if(arg == "Hose")
            {
                CommonHoseSchema.ForEach(Grid.Plot.Remove);
                CommonHoseSchema.Clear();
            }
            else if(arg == "Path")
            {
                CommonPathSchema.ForEach(Grid.Plot.Remove);
                CommonPathSchema.Clear();
            }
            Grid.Refresh();
        }

        private ScottPlot.Plottables.Ellipse DrawPipeSchema(int[] PipeIndex)
        {
            var C = Grid.Plot.Add.Circle(Ctx.step_x * PipeIndex[0], Ctx.step_y * PipeIndex[1], Ctx.radius);
            var S = Ctx.map[$"{PipeIndex[0]}_{PipeIndex[1]}"].state;
            if(S != "Inaccessible")
            {
                if(S != "Finger") C.FillColor = ScottPlot.Colors.Purple;
                else C.FillColor = ScottPlot.Colors.DarkBlue;
            }
            return C;
        }

        private List<ScottPlot.Plottables.Ellipse> PartialSchemas = [];

        public void ClearPartialSchemas()
        {
            PartialSchemas.ForEach(PS => Grid.Plot.Remove(PS));
            Grid.Refresh();
            PartialSchemas.Clear();
        }

        public void DrawPipe(KeyValuePair<int[], List<int[]>> Pipe)
        {
            PartialSchemas.AddRange(Pipe.Value.ConvertAll(I => DrawPipeSchema(I)));
            PartialSchemas.Add(DrawPipeSchema(Pipe.Key));
            Grid.Refresh();
        }

    }

    private RobotModel Robot;

    private UnorientedRobotModel URobot;

    private DataContext Ctx;

    private SchemaRenderer Painter;

    private ElementaryPlanner P;

    public string Path;

    public int[] ObservingPoint;

    private SchemaRenderer TestPainter;

    public AvaPlot SetPlotForTestPainter{set => TestPainter.Grid = value;}

    public MainWindow()
    {
        InitializeComponent();

        using(var S = new StreamReader("Перчатка"))
        {
            Ctx = JsonSerializer.Deserialize<DataContext>(S.ReadToEnd());
        }
        Robot = new(Ctx, [19, 17]);
        URobot = new();
        URobot.Calculate(Ctx, [19, 17]);
        GridPlot.Plot.Axes.SquareUnits();
        Painter = new(GridPlot, Ctx, Robot);
        TestPainter = new(GridPlot, Robot.GetTestCtx, Robot);
        TopLevel.GetTopLevel(this).KeyDown += OnEscKeyDown;
        TopLevel.GetTopLevel(this).KeyDown += ShowCurrPipeHoseable;
        // TestPainter.DrawPipeDesk();
        // TestPainter.DrawSegment();
        // TestPainter.DrawPathConstraints();
    }

    public void OnEscKeyDown(object sender, KeyEventArgs e)
    {
        if(e.Key == Key.Escape) Close(sender, e);
    }

    public void FileOpen(object sender, RoutedEventArgs e) => AllAuxiliaryWindows.Add(new PathInputWindow(){MainWin = this});

    public void Draw(object sender, RoutedEventArgs e)
    {
        if(sender is PathInputWindow)
        {
            using(var S = new StreamReader(Path))
            {
                Ctx = JsonSerializer.Deserialize<DataContext>(S.ReadToEnd());
            }
            Painter.ClearAll();
            Painter = new(GridPlot, Ctx, Robot);
            Painter.DrawPipeDesk();
        }
    }

    public void Close(object sender, RoutedEventArgs e)
    {
        AllAuxiliaryWindows.ForEach(W => {if(W is not null) W.Close();});
        Close();
    }

    private List<Window> AllAuxiliaryWindows = [];

    private SecondPlanner PF;

    public void ShowCalculated(object sender, RoutedEventArgs e)
    {
        // P = new ElementaryPlanner(new DataContextForTestPipeDesk(Ctx, 50));
        // AllAuxiliaryWindows.Add(new SchemaWindow(){MainWin = this});
        // TestPainter.Grid = (AllAuxiliaryWindows.Last() as SchemaWindow).GridPlot;
        // TestPainter.Ctx = P.GetCtx;
        // TestPainter.DrawPipeDesk();
        // if((sender as MenuItem).Name == "Item1") TestPainter.DrawSegment();
        //var PF = new PathFinder(Ctx, 500); //здесь происходит главный расчёт
        PF = new(SomeGridModel: new(Ctx));
        PF.Solve(Convert.ToUInt32(Ctx.map.Keys.Count) / 3);
    }

    private System.Timers.Timer ATimer;

    private int IteratorForPartialDraw = 0;

    private void InitializeTimer(int TimeOffset = 1000)
    {
        ATimer = new(TimeOffset);
        ATimer.Elapsed += OnTimedEvent;
        ATimer.AutoReset = true;
        ATimer.Enabled = true;
    }

    private void OnTimedEvent(object sender, ElapsedEventArgs e) => DrawCurrPipeHoseable();

    public void ShowPartialCalculated(object sender, RoutedEventArgs e)
    {
        InitializeTimer();
        // foreach(var p in PF.HoseIndexesPhiFunc)
        //     Painter.DrawPipe(p);
    }

    public void ShowCurrPipeHoseable(object sender, KeyEventArgs e)
    {
        if(e.Key == Key.Up) DrawCurrPipeHoseable();
    }

    private void DrawCurrPipeHoseable()
    {
         if(IteratorForPartialDraw == PF.HoseIndexesPhiFunc.Count)
        {
            IteratorForPartialDraw = 0;
            Painter.ClearPartialSchemas();
        }
        Painter.DrawPipe(PF.HoseIndexesPhiFunc.ToList()[IteratorForPartialDraw]);
        IteratorForPartialDraw++;
    }

    public void DrawHoseSegment(string Index)
    {
        try{TestPainter.DrawElementaryDiscreteIndex(P.GetIndexes.Find(I => I.Equals(Index)), "Hose");}
        catch{}
    }

    public void DrawPathSegment(string Index)
    {
        try{TestPainter.DrawElementaryDiscreteIndex(P.GetIndexes.Find(I => I.Equals(Index)), "Path");}
        catch{}
    }

    public void DrawCommonHose(string Index)
    {
        URobot = new();
        URobot.Calculate(Robot.GetTestCtx, [19, 17]);
        TestPainter.DrawCommonHoseSchema(URobot.FindPositionsForCurrIndex(StringToIndex(Index), "Hose"), StringToIndex(Index));
    }

    public void DrawCommonPath(string Index)
    {
        URobot = new();
        URobot.Calculate(Robot.GetTestCtx, [19, 17]);
        TestPainter.DrawCommonPathSchema(URobot.FindPositionsForCurrIndex(StringToIndex(Index), "Path"), StringToIndex(Index));
    }

    public void ClearCommonSchema(string arg) => TestPainter.ClearCommon(arg);

    private static int[] StringToIndex(string Name)
    {
        var IndAsSubStrs = Name.Split(' ');
        return [Convert.ToInt32(IndAsSubStrs[0]), Convert.ToInt32(IndAsSubStrs[1])];
    }
}