(TeX-add-style-hook
 "tmp-report"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("scrreprt" "12pt")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("inputenc" "utf8") ("babel" "ngerman") ("fontenc" "T1") ("acronym" "printonlyused") ("hyperref" "breaklinks=true" "colorlinks=true" "citecolor=blue" "menucolor=blue" "urlcolor=blue" "linkcolor=blue")))
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "href")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperref")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperimage")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperbaseurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "nolinkurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
   (TeX-run-style-hooks
    "latex2e"
    "abkv"
    "420_Jetson_Nano"
    "scrreprt"
    "scrreprt12"
    "inputenc"
    "babel"
    "fontenc"
    "acronym"
    ""
    "graphicx"
    "geometry"
    "amsmath"
    "fancyhdr"
    "lmodern"
    "color"
    "subfig"
    "wrapfig"
    "hyperref")
   (TeX-add-symbols
    "figureautorefname")
   (LaTeX-add-bibliographies
    "literatur"))
 :latex)

