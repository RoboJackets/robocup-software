
# Vision

## About

The RoboCup field has two cameras above the field (one over each half), which are connected to a league-provided 'vision computer'.  This computer takes in images from the two cameras (at about 60Hz) and uses the [ssl-vision](https://code.google.com/p/ssl-vision/) program for image processing.  It then sends out (x,y) coordinates of all of the robots and the ball over the network (in protobuf packets over UDP) to the two teams' field computers.

\dot
digraph vision {
	rankdir=LR;

	subgraph cluster_vision_computer {
		label="Vision Computer";

		SSL_Vision [label="ssl-vision"];
	}

	{Camera0, Camera1} -> SSL_Vision [label="Firewire"];

	subgraph cluster_field_computer {
		label="Field Computer";

		soccer;
	}

	SSL_Vision -> soccer [label="protobuf over UDP"];
}
\enddot


## Dot Patterns

Each robot has a dot pattern on top that lets the vision system recognize which team it's on and what robot/shell number it is.  The center dot is yellow or blue to indicate team and the surrounding 4 dots are a sort of binary to distinguish robot number.

Here's an image of all of the dot patterns.  Shell number 0 is the upper-left and they increase as they go right, then down.

![Dot Patterns](dot_patterns.png)

