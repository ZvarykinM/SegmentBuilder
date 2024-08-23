using System;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;

namespace SegmentBuilder;

public partial class PathInputWindow : Avalonia.Controls.Window
{
    public MainWindow MainWin;

    public PathInputWindow()
    {
        InitializeComponent();
        TopLevel.GetTopLevel(this).KeyDown += OnEnterKeyDown;
        Show();
    }

    public void Close(object sender, RoutedEventArgs e)
    {
        MainWin.Path = PathInput.Text ?? "КуАЭС";
        MainWin.Draw(this, new RoutedEventArgs());
        Close();
    }

    public void OnEnterKeyDown(object sender, KeyEventArgs e)
    {
        if(e.Key is Key.Enter) Close(sender, e);
    }
}