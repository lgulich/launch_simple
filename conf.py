"""Configuration file for the Sphinx documentation builder."""
# pylint: disable=invalid-name
import sys

# -- Project information -----------------------------------------------------
project = 'launch_simple'
copyright = '2023, Lionel Gulich'  #pylint: disable=redefined-builtin
author = 'Lionel Gulich'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
sys.path += ['launch_simple']
extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
]

templates_path = ['_templates']
exclude_patterns = [
    '_build',
    'Thumbs.db',
    '.DS_Store',
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

root_doc = 'index'

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
# html_logo = '_static/ascento_logo_white_5000x2000.png'

html_context = {
    'display_github': True,
    'github_user': 'lgulich',
    'github_repo': 'launch_simple',
    'github_version': 'master/',
}
