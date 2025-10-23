# Milestone 1 Report - LaTeX Files

This folder contains all LaTeX files and materials for the Milestone #1 report.

## Main Files

- **milestone1_report.pdf** - Final compiled PDF (ready to submit)
- **milestone1_report.tex** - LaTeX source file
- **figures/** - Contains all plots and figures used in the report
  - `coverage_comparison.pdf` - Automated vs. manual exploration comparison

## Template Files

- **IEEEtran.cls** - RSS conference LaTeX class
- **paper_template.tex** - Original RSS template
- **paper_template.pdf** - Example of template output
- **references.bib** - Bibliography file (minimal citations)

## Compilation

To recompile the PDF after making changes:

```bash
cd latex_report
pdflatex milestone1_report.tex
pdflatex milestone1_report.tex  # Run twice for references
```

## Submission

Submit `milestone1_report.pdf` to Canvas. The report is exactly 3 pages and follows RSS formatting requirements.
