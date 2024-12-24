#import "@preview/cetz:0.3.1": *
#import "@preview/cetz-plot:0.1.0": plot, chart


== Отчет об улучшении производительности (ахах)

=== Specs
Все замеры производились со следующими характеристиками (если не указано иное):
- #image("neofetch.png")
- 1000 тиков
- Время указано в среднем за $N$ запусков, где $N>=10$
- Размер поля: `36x84`
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
    #align(center)[*FIXED(64,8)*]
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

=== Многопоточное улучшение
Упор был сделан на распараллеливание функции `propagate_flow` (см. результаты профилирования).
#stack(
  dir: ltr,
  spacing: 1fr,
  [
    #image("valg_og.png", width: 45%)
    #align(center)[_Fig. 1. OG_]
  ],
  [
    #image("valg_j7.png", width: 45%)
    #align(center)[_Fig. 2. Paralleled (-j7)_]
  ],
)
На картинках выше видно, что накладные расходы на многопоточность ощутимо превышают полезную нагрузку -- это потому, что я не успел додебажить и запустить какой-нибудь огромный тест (исходное решение не сильно приспособлено для больших тестов).

*Параметры тестов*:
- 290x84
- `-j7` всего использует 8 потоков

#box(
  stroke: stroke(1pt),
  inset: 10pt,
  [
    #align(center)[*FIXED(64,8), non-precomp size*]
    #canvas({
      let data = (
        ([-j1], 12000),
        ([-j2], 9000),
        ([-j4], 8100),
        ([-j7], 7000),
      )

      chart.barchart(
        mode: "basic",
        size: (gragh_width, auto),
        x-label: [_Time, ms_],
        x-tick-step: 1500,
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
    #align(center)[*Сравнеие с изначальной программой*]
    #canvas({
      let data = (
        ([-j1], 12000),
        ([-j2], 9000),
        ([-j4], 8100),
        ([-j7], 7000),
        ([OG], 20250),
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