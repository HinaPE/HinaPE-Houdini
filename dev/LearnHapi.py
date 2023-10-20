import hou

node = hou.pwd()
geo = node.geometry()

# Add code to modify contents of geo.
# Use drop down menu to select examples.

def print_tree(node, indent=0):
	for child in node.children():
		print("  " * indent + child.name())
		print_tree(child, indent + 3)

hou.hipFile.clear()
