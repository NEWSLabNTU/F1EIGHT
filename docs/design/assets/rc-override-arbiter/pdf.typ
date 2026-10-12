// Renders ../../rc-override-arbiter.md to ../../rc-override-arbiter.pdf with its figures.
//   typst compile --root docs/design docs/design/assets/rc-override-arbiter/pdf.typ docs/design/rc-override-arbiter.pdf
#import "@preview/cmarker:0.1.6"
#set page(paper: "a4", margin: (x: 18mm, y: 18mm), numbering: "1")
#set text(font: ("DejaVu Sans"), size: 9.5pt)
#show raw: set text(font: "DejaVu Sans Mono", size: 8.5pt)
#show heading.where(level: 1): set text(size: 17pt)
#show heading.where(level: 2): it => { v(6pt); set text(size: 13pt, fill: rgb("#1f4e79")); it }
#set table(stroke: 0.4pt + rgb("#9aa5b1"), inset: 4pt)
#show table: set text(size: 8.5pt)
#show link: set text(fill: rgb("#1f4e79"))
#let fig(path, alt) = {
  let local = path.replace("assets/rc-override-arbiter/", "")
  if local == "pins.svg" {
    page(paper: "a3", flipped: true, margin: 10mm,
      align(horizon, figure(image(local, width: 100%), caption: alt)))
  } else { figure(image(local, width: 100%), caption: alt) }
}
#cmarker.render(read("../../rc-override-arbiter.md"),
  scope: (image: (path, alt: none) => fig(path, alt)))
