using System;
using System.Collections.Generic;
using System.Linq;
using DiscreteRobotImplementation;
using Microsoft.Z3;

namespace SegmentBuilder;

class DiscreteIndex(RobotModel SomeRobot, int[] IndexPair, DataContext SomeCtx, Context LogicCtx)
{
    private readonly RobotModel Robot = SomeRobot;

    private readonly int[] Index = IndexPair;

    private readonly DataContext Ctx = SomeCtx;

    private Context Logic = LogicCtx;

    public string NameInCtx => $"{Index[0]}_{Index[1]}";

    public BoolExpr FUExpression => Logic.MkBoolConst($"FU{Index[0]}_{Index[1]}");

    public BoolExpr FLExpression => Logic.MkBoolConst($"FL{Index[0]}_{Index[1]}");

    public BoolExpr HExpression => Logic.MkBoolConst($"H{Index[0]}_{Index[1]}");

    private List<BoolExpr> GenerateConstraintVariables(string Param)
    {
        var HIndexes = new List<int[]>();
        (List<int[]> CountedHIndexes, string Name) = Param switch
        {
            "H_Upper" => (Robot.HIndexesUpper, "H"),   //для вызова в качестве fi_u
            "H_Lower" => (Robot.HIndexesLower, "H"), //для вызова в качестве fi_l
            "F0_Lower" => (Robot.F0IndexesLower, "FL"),  //для вызова в качестве fi_u_^{-1}
            "F0_Upper" => (Robot.F0IndexesUpper, "FU"),  //для вызова в качестве fi_l_^{-1}
            _ => throw new Exception("НЕИЗВЕСТНЫЙ ПАРАМЕТР ВЫБОРА")
        };
        CountedHIndexes.ForEach(I => HIndexes.Add([I[0] - Robot.GetF0[0] + Index[0], I[1] - Robot.GetF0[1] + Index[1]]));
        var Buffer = HIndexes.ConvertAll(I => $"{I[0]}_{I[1]}").FindAll(Name => Ctx.map.Keys.ToList().Contains(Name) && Ctx.map[Name].state != "Inaccessible");
        return Buffer.ConvertAll(I => Logic.MkBoolConst(Name + I));
    }

    public BoolExpr HoseUpperConstratint => Logic.MkImplies(FUExpression, Logic.MkAnd(GenerateConstraintVariables("H_Upper")));

    public BoolExpr HoseLowerConstraint => Logic.MkImplies(FLExpression, Logic.MkAnd(GenerateConstraintVariables("H_Lower")));

    public BoolExpr PathConstraint
    {
        get
        {
            var HVars = GenerateConstraintVariables("F0_Lower");
            HVars.AddRange(GenerateConstraintVariables("F0_Upper"));
            return Logic.MkImplies(HExpression, Logic.MkAtLeast(HVars, 1));
        }
    }

    private bool FUVal, FLVal, HVal;

    public bool FU_IsCorrect
    {
        private set => FUVal = value; 
        get => FUVal;
    }

    public bool FL_IsCorrect
    {
        private set => FLVal = value;
        get => FLVal;
    }

    public bool H_IsCorrect
    {
        private set => HVal = value;
        get => HVal;
    }

    public bool SetFUVal(Solver SomeSolver) => FUVal = SomeSolver.Model.Evaluate(FUExpression).ToString() == "true";

    public bool SetFLVal(Solver SomeSolver) => FLVal = SomeSolver.Model.Evaluate(FLExpression).ToString() == "true"; 

    public bool SetHVal(Solver SomeSolver) => HVal = SomeSolver.Model.Evaluate(HExpression).ToString() == "true";
}

class Planner
{
    private readonly DataContext Ctx;

    private readonly Context Logic;

    private readonly Solver Controller;

    private List<DiscreteIndex> IndexesOfPipeDesk = [];

    public Planner(DataContext SomeDataContext)
    {
        Ctx = SomeDataContext;
        var Robot = new RobotModel(Ctx, [20, 20]);
        using(Logic = new())
        {
            Ctx.map.Values.ToList().ForEach(Pipe => IndexesOfPipeDesk.Add(new(Robot, Pipe.index, Ctx, Logic)));
            var CommonUpperHoseConstr = Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.HoseUpperConstratint));
            var CommonLowerHoseConstr = Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.HoseLowerConstraint));
            var CommonPathConstr = Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.PathConstraint));
            var Goal = Logic.MkEq(Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.HExpression)), Logic.MkBool(true));
            Controller = Logic.MkSolver();
            Controller.Add(Logic.MkAnd([CommonLowerHoseConstr, /*CommonUpperHoseConstr,*/ CommonPathConstr, Goal]));
            IndexesOfPipeDesk.ForEach(I => {I.SetFLVal(Controller); I.SetFUVal(Controller); I.SetHVal(Controller);});
        }
        Console.WriteLine(IndexesOfPipeDesk.Count(I => I.FL_IsCorrect));
        Console.WriteLine(IndexesOfPipeDesk.Count(I => I.FU_IsCorrect));
        Console.WriteLine(IndexesOfPipeDesk.Count(I => I.H_IsCorrect));
    }
}

class ElementaryDiscreteIndex(RobotModel SomeRobot, int[] IndexPair, DataContext SomeCtx, Context LogicCtx): IEquatable<string>
{
    public bool Equals(string SomeIndex) => NameInCtx == SomeIndex;

    private RobotModel Robot = SomeRobot;

    private int[] Index = IndexPair;

    public int[] GetIndexValue => Index;

    private DataContext Ctx = SomeCtx;

    private Context Logic = LogicCtx;

    public string NameInCtx => $"{Index[0]}_{Index[1]}";

    public BoolExpr FExpr => Logic.MkBoolConst("F" + NameInCtx);

    public BoolExpr HExpr => Logic.MkBoolConst("H" + NameInCtx);

    public List<int[]> HIndexes => Robot.GetIndexes(Index, "HLower");

    public List<int[]> FIndexes => Robot.GetIndexes(Index, "F0Lower");
    
    public List<BoolExpr> HVars => HIndexes.ConvertAll(S => Logic.MkBoolConst("H" + S));

    public List<BoolExpr> FVars => FIndexes.ConvertAll(S => Logic.MkBoolConst("F" + S));

    public BoolExpr HoseConstr => Logic.MkImplies(FExpr, Logic.MkAnd(HVars));

    public BoolExpr PathConstr => Logic.MkImplies(HExpr, Logic.MkAtLeast(FVars, 1));

    public void SetState(Solver BoolCalculator)
    {
        if(BoolCalculator.Model.Evaluate(HExpr).ToString() == "true") Ctx.map[NameInCtx].state = "Done";
        if(BoolCalculator.Model.Evaluate(FExpr).ToString() == "true") Ctx.map[NameInCtx].state = "Finger";
    }
}

class ElementaryPlanner
{
    private readonly DataContext Ctx;

    public DataContext GetCtx => Ctx;

    private readonly Context Logic;

    private readonly Solver Controller;

    private readonly RobotModel Robot;

    private readonly List<ElementaryDiscreteIndex> IndexesOfPipeDesk = [];

    public List<ElementaryDiscreteIndex> GetIndexes => IndexesOfPipeDesk;

    private readonly BoolExpr CommonHoseConstr;
    private readonly BoolExpr CommonPathConstr;
    private readonly BoolExpr Goal;

    public DataContext GetTestPipeConfiguration => Robot.GetTestCtx;

    public ElementaryPlanner(DataContext SomeDataContext)
    {
        Ctx = SomeDataContext;
        Robot = new RobotModel(Ctx, [20, 20]);
        using(Logic = new())
        {
            Ctx.map.Values.ToList().ForEach(Pipe => IndexesOfPipeDesk.Add(new(Robot, Pipe.index, Ctx, Logic)));
            CommonHoseConstr = Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.HoseConstr));
            CommonPathConstr = Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.PathConstr));
            Goal = Logic.MkAtLeast(IndexesOfPipeDesk.ConvertAll(I => I.HExpr), 30);//Logic.MkEq(Logic.MkAnd(IndexesOfPipeDesk.ConvertAll(I => I.HExpr)), Logic.MkBool(true));
            Controller = Logic.MkSolver();
        }
    }

    public void SetVariableValues()
    {
        try
        {
            Controller.Add(Logic.MkAnd([CommonHoseConstr, CommonPathConstr, Goal]));
            IndexesOfPipeDesk.ForEach(I => I.SetState(Controller));
        }
        catch{Console.WriteLine("МОДЕЛЬ НЕ СУЩЕСТВУЕТ");}
    }
}