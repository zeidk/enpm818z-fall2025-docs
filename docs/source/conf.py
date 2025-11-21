import os, sys
from datetime import date
import json

project = "ENPM818Z Fall 2025"
author = "Z. Kootbally"
copyright = f"{date.today().year}, {author}"
release = "v1.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autosummary",
    "sphinxcontrib.mermaid",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx_design",
    "sphinx_proof",
    "sphinx.ext.todo",
    "sphinx.ext.mathjax",
    "sphinx.ext.viewcode",
]


# Prerender options for better performance
katex_prerender = True


intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}
proof_numbered = {
    "theorem": True,
    "lemma": True,
    "algorithm": True,
    "example": False,
}

todo_include_todos = True

templates_path = ["_templates"]
exclude_patterns = []
# html_theme = 'sphinx_rtd_theme'
html_theme = "furo"
# Furo-specific options (all optional)
html_theme_options = {
    "sidebar_hide_name": True,  # hides large project name in sidebar
    "source_repository": "https://github.com/zeidk/enpm818z-fall-2025",
    "source_branch": "main",
    # "source_directory": "docs/source/",
    "light_logo": "enpm818Z-light.png",
    "dark_logo": "enpm818Z-dark.png",
}

numfig = True
numfig_format = {
    "pseudocode": "Algorithm %s"  # This is what sphinxcontrib-pseudocode uses
}

html_static_path = ["_static"]
master_doc = "index"

html_css_files = [
    "custom.css",
    "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css",
]
