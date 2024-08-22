using System;
using Avalonia.Controls;
using Avalonia.Interactivity;

namespace SegmentBuilder;

public partial class SchemaWindow : Avalonia.Controls.Window
{
    public MainWindow MainWin;

    public SchemaWindow()
    {
        InitializeComponent();
        Show();
    }
}