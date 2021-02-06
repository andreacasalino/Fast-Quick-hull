color 0b

del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl *.lot *.lof
"./content/pdf_tex_fixer/x64/Release/pdf_tex_fixer.exe" "./content/image_folders" "./content/last_compilation_event"

pdflatex Fast_QHull.tex
pdflatex Fast_QHull.tex

del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl *.lot *.lof