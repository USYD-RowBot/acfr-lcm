function [] = parseAutocal(boardWidth, boardHeight, squareSize, outfile)

    if ~exist('squareSize')
        squareSize = 0.03;
    end
    if ~exist('boardWidth')
        boardWidth = 7 - 1;
    end
    if ~exist('boardHeight')
        boardHeight = 12 - 1;
    end
    if ~exist('outfile')
        outfile  = 'autocalResult.mat';
    end

    boardWidth   = boardWidth *squareSize;
    boardHeight  = boardHeight*squareSize;

    translations = readMatFromXml('ExtrTrans.xml');
    rotations    = readMatFromXml('ExtrRotation.xml');
    K            = readMatFromXml('Intrinsics.xml'); %Internal cam params
    distortion   = readMatFromXml('Distortion.xml');

    x    = [0 boardWidth boardWidth    0            0];
    y    = [0 0          boardHeight   boardHeight  0];
    z    = [0 0          0             0            0];
    GRID = [x; y; z; ones(1,5)];

    plot3(GRID(1,:), GRID(2,:), GRID(3,:), '-m', 'LineWidth', 3);
    plot_coordinate_frame(eye(3), [0 0 0]', '0', .1, 'rgb');

    numPoses = size(translations, 2);
    
    XTargetCamera = zeros(6, numPoses);
    XCameraTarget = zeros(6, numPoses);
    PMats = zeros(3, 4, numPoses);
    
    for i=1:numPoses
        
        %Autocal gives us 6DOF from camera to target, which is
        %needed for camera matrix
        t                  = translations(:,i);
        rod                = rotations(:,i);
        R                  = rodrigues(rod);
        rph                = rot2rph(R);
        XCameraTarget(:,i) = [t; rph];
        
        %...but we want "target to camera" to plot the frames wrt target
        XTargetCamera(:,i) = ssc_inverse(XCameraTarget(:,i));
        x_tc               = XTargetCamera(:,i);
        RPlot              = rotxyz(x_tc(4:6));
        tPlot              = x_tc(1:3);

        plot_coordinate_frame(RPlot, tPlot, num2str(i), .05, 'rgb');
        P = K*[R t];
        PMats(:,:,i) = P;
        
    end

    save(outfile, 'XTargetCamera', 'K', 'distortion', 'PMats');

    axis equal
    grid on
    xlabel('x (meters)')
    ylabel('y (meters)')
    zlabel('z (meters)')
    camproj('perspective')

function mat = readMatFromXml(filename)
    matXml     = parseXml(filename);
    matValue   = matXml.Children(2).Children(8).Children.Data;
    matValue   = regexprep(matValue, '\s+', ' ');
    matData    = sscanf(matValue, '%f');
    matRows    = str2num(matXml.Children(2).Children(2).Children.Data);
    matCols    = str2num(matXml.Children(2).Children(4).Children.Data);
    mat        = reshape(matData, matCols, matRows);
    mat        = mat';
    
function theStruct = parseXml(filename)
% PARSEXML Convert XML file to a MATLAB structure.
    try
        tree = xmlread(filename);
    catch
        error('Failed to read XML file %s.',filename);
    end

    % Recurse over child nodes. This could run into problems 
    % with very deeply nested trees.
    try
        theStruct = parseChildNodes(tree);
    catch
        error('Unable to parse XML file %s.',filename);
    end

function children = parseChildNodes(theNode)
% Recurse over node children.
    children = [];
    if theNode.hasChildNodes
        childNodes = theNode.getChildNodes;
        numChildNodes = childNodes.getLength;
        allocCell = cell(1, numChildNodes);

        children = struct(             ...
            'Name', allocCell, 'Attributes', allocCell,    ...
            'Data', allocCell, 'Children', allocCell);

        for count = 1:numChildNodes
            theChild = childNodes.item(count-1);
            children(count) = makeStructFromNode(theChild);
        end
    end

function nodeStruct = makeStructFromNode(theNode)
% Create structure of node info.

    nodeStruct = struct(                        ...
        'Name', char(theNode.getNodeName),       ...
        'Attributes', parseAttributes(theNode),  ...
        'Data', '',                              ...
        'Children', parseChildNodes(theNode));

    if any(strcmp(methods(theNode), 'getData'))
            nodeStruct.Data = char(theNode.getData); 
    else
        nodeStruct.Data = '';
    end

function attributes = parseAttributes(theNode)
% Create attributes structure.

    attributes = [];
    if theNode.hasAttributes
        theAttributes = theNode.getAttributes;
        numAttributes = theAttributes.getLength;
        allocCell = cell(1, numAttributes);
        attributes = struct('Name', allocCell, 'Value', ...
                            allocCell);

        for count = 1:numAttributes
            attrib = theAttributes.item(count-1);
            attributes(count).Name = char(attrib.getName);
            attributes(count).Value = char(attrib.getValue);
        end
    end
    
function coord = homoToNonhomo(x)
    
    coord = x ./ x(end);
    coord = coord(1:end-1);
