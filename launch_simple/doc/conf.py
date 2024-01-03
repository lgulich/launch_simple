"""Configuration file for the Sphinx documentation builder."""
# pylint: disable=invalid-name

# -- Project information -----------------------------------------------------
project = 'launch_simple'
copyright = '2023, Lionel Gulich'  #pylint: disable=redefined-builtin
author = 'Lionel Gulich'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
nitpicky = True
nitpick_ignore = [
    ('py:class', 'launch_ros.descriptions.composable_node.ComposableNode'),
    ('py:class', 'launch.launch_description.LaunchDescription'),
]

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
html_static_path = []

html_context = {
    'display_github': True,
    'github_user': 'lgulich',
    'github_repo': 'launch_simple',
    'github_version': 'master/',
}
