using System;
using Avalonia.Controls;
using Avalonia.Interactivity;

namespace SegmentBuilder;

public partial class PathInputWindow : Avalonia.Controls.Window
{
    public MainWindow MainWin;

    public PathInputWindow()
    {
        InitializeComponent();
        Show();
    }

    public void Close(object sender, RoutedEventArgs e)
    {
        MainWin.Path = PathInput.Text ?? "КуАЭС";
        MainWin.Draw(this, new RoutedEventArgs());
        Close();
    }
}