using System;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using DiscreteRobotImplementation;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace SegmentBuilder;

public partial class SchemaWindow : Avalonia.Controls.Window
{
    public MainWindow MainWin;
    
    public SchemaWindow()
    {
        InitializeComponent();
        GridPlot.Plot.Axes.SquareUnits();
        TopLevel.GetTopLevel(this).KeyDown += OnKeyDown;
        Show();
    }

    public void DrawHoseSegment(object sender, RoutedEventArgs e)
    {
        if((bool)Check.IsChecked) MainWin.DrawCommonHose(CoordInput.Text);
        else MainWin.DrawHoseSegment(CoordInput.Text);
    }

    public void DrawPathSegment(object sender, RoutedEventArgs e)
    {
        if((bool)Check.IsChecked) MainWin.DrawCommonPath(CoordInput.Text);
        else MainWin.DrawPathSegment(CoordInput.Text);
    }

    public void OnKeyDown(object sender, KeyEventArgs e)
    {   
        switch(e.Key)
        {
            case Key.Left: 
                MainWin.ClearCommonSchema("Hose");
                break;
            case Key.Down:
                MainWin.ClearCommonSchema("Path");
                break;
            case Key.Right:
                MainWin.DrawCommonHose(CoordInput.Text);
                break;
            case Key.Up:
                MainWin.DrawCommonPath(CoordInput.Text);
                break;
        }
        // if(e.Key is Key.Left) MainWin.ClearCommonSchema("Hose");
        // else if(e.Key is Key.Down) MainWin.ClearCommonSchema("Path");
        
    }
}