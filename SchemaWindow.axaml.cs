using System;
using Avalonia.Controls;
using Avalonia.Interactivity;
using DiscreteRobotImplementation;

namespace SegmentBuilder;

public partial class SchemaWindow : Avalonia.Controls.Window
{
    public MainWindow MainWin;
    
    public SchemaWindow()
    {
        InitializeComponent();
        GridPlot.Plot.Axes.SquareUnits();
        Show();
    }

    public void DrawHoseSegment(object sender, RoutedEventArgs e) => MainWin.DrawHoseSegment(CoordInput.Text);

    public void DrawPathSegment(object sender, RoutedEventArgs e) => MainWin.DrawPathSegment(CoordInput.Text);
}