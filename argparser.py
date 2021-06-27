#importing argparse module
import argparse

# create a keyvalue class
class keyvalue(argparse.Action):
	# Constructor calling
	def __call__( self , parser, namespace,
				values, option_string = None):
		setattr(namespace, self.dest, dict())
		
		for value in values:
			# split it into key and value
			key, value = value.split('=')
			# assign into dictionary
			getattr(namespace, self.dest)[key] = value

# creating parser pbject
parser = argparse.ArgumentParser()

# adding an arguments
parser.add_argument('--kwargs',
					nargs='*',
					action = keyvalue)

#parsing arguments
args = parser.parse_args()

# show the dictionary
if(args.kwargs.has_key("one")):
	print(args.kwargs["one"])

