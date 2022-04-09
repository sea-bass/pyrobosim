""" Utilities to export worlds to Gazebo """

import os
import shutil
import itertools
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import split

from ..utils.general import get_data_folder, replace_special_yaml_tokens


class WorldGazeboExporter:
    def __init__(self, world):
        self.world = world

        # Set up template text for string replacement.
        self.data_folder = get_data_folder()
        self.template_folder = os.path.join(self.data_folder, "templates")
        self.model_template_text = self.read_template_file(
            "model_template.sdf")
        self.model_config_template_text = self.read_template_file(
            "model_template.config")
        self.link_template_text = self.read_template_file(
            "link_template_polyline.sdf")

    def export(self):
        """ Exports the world to an SDF file to use with Gazebo """
        world_name = self.world.name
        if world_name is None:
            world_name = "gen_world"

        # Set up text to export
        world_text = self.read_template_file("world_template.sdf")
        self.model_include_text = ""

        # Define output folder
        self.out_folder = os.path.join(self.data_folder, "worlds", world_name)
        self.include_model_paths = set([self.out_folder])

        # Convert all the world entities for export
        self.create_walls_for_export()
        self.create_locations_and_objects_for_export()

        # Add all the model includes to the world file and write it.
        world_text = world_text.replace(
            "$MODEL_INCLUDES", self.model_include_text)
        world_file_name = os.path.join(self.out_folder, f"{world_name}.sdf")
        with open(world_file_name, "w") as f:
            f.write(world_text)

        # Print commands for the user to start the world.
        # TODO: We could generate a script to do this instead?
        print(f"\nWorld file saved to {world_file_name}\n")
        print(f"Ensure to update your Gazebo model path:")
        include_path_str = ":".join(self.include_model_paths)
        print(f"    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{include_path_str}\n")
        print(f"To start the world, enter")
        print(f"    gazebo {world_file_name}\n")
        return

    def create_walls_for_export(self, walls_name="walls"):
        """ Convert all room / hallway polygons to Gazebo representations """
        # Build up a list of links representing each wall segment in the world.
        full_links_text = ""
        for obj in itertools.chain(self.world.rooms, self.world.hallways):
            full_links_text += self.create_sdf_link_text(
                self.link_template_text, obj, "walls")

        # Create a model SDF file and tack on the links text created above.
        walls_text = self.model_template_text
        walls_text = walls_text.replace("$NAME", walls_name)
        walls_text = walls_text.replace("$POSE", "0 0 0 0 0 0")
        walls_text = walls_text.replace("$STATIC", "1")
        walls_text = walls_text.replace("$LINKS", full_links_text)
        walls_folder = os.path.join(self.out_folder, "walls")
        if os.path.isdir(self.out_folder):
            shutil.rmtree(self.out_folder)
        os.makedirs(walls_folder)
        with open(os.path.join(self.out_folder, walls_name, "model.sdf"), "w") as f:
            f.write(walls_text)

        # Create a config file.
        config_text = self.model_config_template_text.replace(
            "$NAME", walls_name)
        with open(os.path.join(self.out_folder, walls_name, "model.config"), "w") as f:
            f.write(config_text)

        # Now include the walls model in the world file.
        self.model_include_text += " "*4 + "<include>\n"
        self.model_include_text += " "*8 + f"<uri>model://{walls_name}</uri>\n"
        self.model_include_text += " "*4 + "</include>\n"

    def create_locations_and_objects_for_export(self):
        """ Export locations and objects to Gazebo representations """
        for entity in itertools.chain(self.world.locations, self.world.objects):
            # If the object category does not have meshes, create a new model and extrude a polyline.
            if entity.metadata["footprint"]["type"] != "mesh":
                folder = os.path.join(self.out_folder, entity.category)
                if not os.path.isdir(folder):
                    os.makedirs(folder)

                    # Create a model SDF file.
                    loc_model_text = self.model_template_text
                    loc_model_text = loc_model_text.replace(
                        "$NAME", entity.category)
                    loc_model_text = loc_model_text.replace(
                        "$POSE", "0 0 0 0 0 0")
                    loc_link_text = self.create_sdf_link_text(
                        self.link_template_text, entity, "object")
                    loc_model_text = loc_model_text.replace(
                        "$LINKS", loc_link_text)
                    loc_model_text = loc_model_text.replace("$STATIC", "0")
                    with open(os.path.join(self.out_folder, entity.category, "model.sdf"), "w") as f:
                        f.write(loc_model_text)

                    # Create a config file.
                    config_text = self.model_config_template_text.replace(
                        "$NAME", entity.category)
                    with open(os.path.join(self.out_folder, entity.category, "model.config"), "w") as f:
                        f.write(config_text)

            # If the entity is based on a Gazebo model, the model can be included as is.
            if entity.metadata["footprint"]["type"] == "mesh":
                model_path = replace_special_yaml_tokens(
                    entity.metadata["footprint"]["model_path"])
                model_path_split = os.path.split(model_path)
                self.include_model_paths.add(model_path_split[0])
                model_name = model_path_split[-1]
            else:
                model_name = entity.category

            # Regardless of the type of entity, add the object instance to the world.
            self.model_include_text += " "*4 + "<include>\n"
            self.model_include_text += " "*8 + \
                f"<uri>model://{model_name}</uri>\n"
            self.model_include_text += " "*8 + f"<name>{entity.name}</name>\n"
            pose_str = f"{entity.pose.x} {entity.pose.y} {entity.pose.z} 0 0 {entity.pose.yaw}"
            self.model_include_text += " "*8 + f"<pose>{pose_str}</pose>\n"
            self.model_include_text += " "*4 + "</include>\n"

    def create_sdf_link_text(self, template_text, entity, entity_type):
        """ Creates SDF link text from a world entity. """

        # Check the height
        if entity_type == "walls":
            height = self.world.wall_height
            poly = entity.viz_polygon
        elif entity_type == "object":
            height = entity.height
            poly = entity.get_raw_polygon()
        else:
            raise Exception(f"Invalid option specified: {entity_type}")

        # Convert everything to a MultiPolygon for consistency
        if isinstance(poly, Polygon):
            polys = MultiPolygon([poly])
        else:
            polys = poly
        polys = [g for g in polys.geoms]

        # If the polygon is a closed ring (i.e. has interiors), split it into
        # two parts along the bounds diagonal to work in Gazebo
        # NOTE: This was done since using full closed polygons causes errors in Gazebo
        for poly in polys:
            if len(poly.interiors) > 0:
                xmin, ymin, xmax, ymax = poly.bounds
                div_line = LineString([(xmin-1, ymin-1), (xmax+1, ymax+1)])
                new_polys = split(poly, div_line)
                polys.remove(poly)
                polys.extend([p for p in new_polys])

        # Now create the SDF text for each polygon in the list
        full_text = ""
        color_str = " ".join([str(c) for c in entity.viz_color])
        for i, poly in enumerate(polys):
            link_text = template_text
            link_name = entity.name
            if i > 0:
                link_name += f"_{i}"
            link_text = link_text.replace("$LINK_NAME", link_name)
            link_text = link_text.replace("$HEIGHT", f"{height:.3}")

            # Write all the polygon coordinates to SDF
            wall_points = ""
            for p in poly.exterior.coords:
                wall_points += " "*12 + f"<point>{p[0]:.3} {p[1]:.3}</point>\n"
            link_text = link_text.replace("$POINTS", wall_points)

            # Set the wall color
            link_text = link_text.replace("$COLOR", color_str)

            # Add the individual link's text to the full text
            full_text += link_text

        return full_text

    def read_template_file(self, filename):
        """ Utility to read a file from the template folder """
        fullfile = os.path.join(self.template_folder, filename)
        with open(fullfile, "r") as f:
            return f.read()
