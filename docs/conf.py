import os
import sys
sys.path.insert(0, os.path.abspath('../moveit2_sdk_python'))

project = 'MoveIt2 SDK Python'
copyright = '2023, Project Contributors'
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'm2r2',
]
html_theme = 'sphinx_rtd_theme'

# Configure autodoc to automatically generate documentation from docstrings
autodoc_member_order = 'bysource'

# Ensure that __init__ methods are documented
autoclass_content = 'both'

# Add mappings for intersphinx to link to other projects' documentation
intersphinx_mapping = {
    'python': ('https.docs.python.org/3', None),
    'rclpy': ('https.docs.ros.org/en/rolling/p/rclpy/', None),
    # Add other mappings if your project depends on other libraries
    # that have Sphinx documentation.
}

# Napoleon settings (if you decide to use Google or NumPy style docstrings)
napoleon_google_docstring = True
napoleon_numpy_docstring = True # Set to False if you are strictly using Google style
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True

# Source file suffixes
source_suffix = {
    '.rst': None,
    '.md': None,
}
