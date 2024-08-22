using Avalonia.Controls;
using DiscreteRobotImplementation;
using System.IO;
using System.Text.Json;
using ScottPlot.Avalonia;
using System.Collections.Generic;
using System.Drawing;
using KVP = System.Collections.Generic.KeyValuePair<string, ScottPlot.Color>;
using System.IO.Pipelines;
using System;
using Avalonia.Interactivity;
using Avalonia.Controls.ApplicationLifetimes;
using System.Linq;

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
            PipeSchemas.Add(Name, C);
            Labels.Add(Name, Text);
        }

        public void DrawPipeDesk()
        {
            foreach(var p in  Ctx.map)
                DrawPipe(p.Key, p.Value);
        }

        private List<ScottPlot.Plottables.Ellipse> HSegment = [];

        private List<ScottPlot.Plottables.Ellipse> F0Segment = [];

        private ScottPlot.Plottables.Ellipse IndexToCircle(int[] Index, string Param)
        {
            var C = Grid.Plot.Add.Circle(Index[0] * Ctx.step_x, Index[1] * Ctx.step_y, Ctx.radius);
            switch(Param)
            {
                case "HU":
                    C.FillColor = ScottPlot.Colors.OrangeRed;
                    break;
                case "HL":
                    C.FillColor = ScottPlot.Colors.OliveDrab;
                    break;
                case "F0":
                    C.FillColor = ScottPlot.Colors.DarkBlue;
                    break;
                case "F2":
                    C.FillColor = ScottPlot.Colors.DarkRed;
                    break;
            }
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
            ClearAll();
            DrawPipeDesk();
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
        }
    }


    private RobotModel Robot;

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
        GridPlot.Plot.Axes.SquareUnits();
        Painter = new(GridPlot, Ctx, Robot);
        TestPainter = new(this.Find<AvaPlot>("GridPlot"), Robot.GetTestCtx, Robot);
        // TestPainter.DrawPipeDesk();
        // TestPainter.DrawSegment();
        // TestPainter.DrawPathConstraints();
    }

    public void FileOpen(object sender, RoutedEventArgs e) => new PathInputWindow(){MainWin = this};

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

    public void Close(object sender, RoutedEventArgs e) => Close();

    public void ShowCalculated(object sender, RoutedEventArgs e)
    {
        switch((sender as MenuItem).Name)
        {
            case "Item0":
                P = new ElementaryPlanner(new DataContextForTestPipeDesk(Ctx, 50));
                Painter.Ctx = P.GetCtx;
                Painter.DrawPipeDesk();
                break;
            case "Item1":
                P = new ElementaryPlanner(new DataContextForTestPipeDesk(Ctx, 50));
                TestPainter.Grid = new SchemaWindow(){MainWin = this}.GridPlot;
                TestPainter.DrawPipeDesk();
                TestPainter.DrawSegment();
                break;
        }
    }

    public void DrawHoseSegment(string Index)
    {
        Console.WriteLine(P is not null);
        TestPainter.DrawElementaryDiscreteIndex(P.GetIndexes.Find(I => I.Equals(Index)), "Hose");
    }

    public void DrawPathSegment(string Index) => TestPainter.DrawElementaryDiscreteIndex(P.GetIndexes.Find(I => I.Equals(Index)), "Path");

    private static int[] StringToIndex(string Name)
    {
        var IndAsSubStrs = Name.Split(' ');
        return [Convert.ToInt32(IndAsSubStrs[0]), Convert.ToInt32(IndAsSubStrs[1])];
    }
}