#import "@preview/cetz:0.3.1": *
#import "@preview/cetz-plot:0.1.0": plot, chart


== Отчет об улучшении производительности (ахах)

=== Specs
Все замеры производились со следующими характеристиками:
- #image("neofetch.png")
- 1000 тиков
- Время указано в среднем за $N$ запусков, где $N>=10$
- Размер поля: `36x84`, если не указано иное
- `CMAKE_BUILD_TYPE`: `Release`
- Компилятор: `clang 18.1.8`

=== Однопоточное улучшение

#let gragh_width = 12.5

#box(
  stroke: stroke(1pt),
  inset: 10pt,
  [
    #align(center)[*FAST_FIXED(32,16), precomp size*]
    #canvas({
      let data = (
        ([OG], 20250),
        ([dyn-types], 16500),
      )

      chart.barchart(
        mode: "basic",
        size: (gragh_width, auto),
        x-label: [_Time, ms_],
        x-tick-step: 2000,
        bar-style: palette.turquoise,
        data,
      )
    })
  ],
)

#box(
  stroke: stroke(1pt),
  inset: 10pt,
  [
    #align(center)[*FIXED(64,8), precomp size*]
    #canvas({
      let data = (
        ([OG], 20250),
        ([dyn-types], 600),
      )

      chart.barchart(
        mode: "basic",
        size: (gragh_width, auto),
        x-label: [_Time, ms_],
        x-tick-step: 2000,
        bar-style: palette.turquoise,
        data,
      )
    })
  ],
)

#box(
  stroke: stroke(1pt),
  inset: 10pt,
  [
    #align(center)[*FIXED(64,8), precomp vs non-precomp*]
    #canvas({
      let data = (
        ([precomp], 600),
        ([non-precomp], 1200),
      )

      chart.barchart(
        mode: "basic",
        size: (gragh_width, auto),
        x-label: [_Time, ms_],
        x-tick-step: 200,
        bar-style: palette.turquoise,
        data,
      )
    })
  ],
)
