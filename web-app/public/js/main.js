$(document).ready(function () {
 
  $.jqplot.config.enablePlugins = true;
 
  s1 = [[1,50],[2, -50],[3, 0],[4, 0]];
 
  plot1 = $.jqplot('chart1',[s1],{
     title: 'Highlighting, Dragging, Cursor and Trend Line',
     axes: {
         xaxis: {
            min: 0,
            max: 16,
            tickOptions: {
                 formatString: '%.2f'
             },
             numberTicks: 4
         },
         yaxis: {
            min: -100,
            max: 100,
             tickOptions: {
                 formatString: '%.2f'
             }
         }
     },
     highlighter: {
         sizeAdjust: 10,
         tooltipLocation: 'n',
         tooltipAxes: 'y',
         tooltipFormatString: '%.2f',
         useAxesFormatters: false
     },
     cursor: {
         show: true
     }
  });
  $.jqplot.Dragable.contraintTo = 'y';
});