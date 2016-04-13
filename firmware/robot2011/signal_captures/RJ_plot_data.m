function fig_handles = RJ_plot_data( data, input_title )

% Determine the number of plots there are in the data set
nbr_plots = size(data,2);

% Divide by the number of subplots that are on a given figure
plotperpage = 4;

nbr_pages = ceil(nbr_plots/plotperpage);

% Initialize the array that will hold the handles to all the figures
fig_handles = gobjects(nbr_pages,1);

% Create evenly distributed pages of output graphs with 4 subplots/figure
for i=1:nbr_pages
    start_index = plotperpage*(i-1)+1;
    end_index = plotperpage*i;
    
    if plotperpage*i > nbr_plots
        end_index = nbr_plots; 
    end
    
    % Copy subset of the data for 1 page
    sending_data = data(:,start_index:end_index);
    
    % Create a new figure that will print on letter paper
    fig_handles(i) = figure('Name', input_title, 'Visible', 'off', 'PaperType', 'usletter');
    
    % Create the page of graphs
    RJ_plot_page(sending_data, plotperpage, fig_handles(i));
    
    clearvars('sending_data');
end

end


function RJ_plot_page( subdata, nbr_subplots, fig_hdl )

% Get the heading names of each subplot
plot_headings(1:size(subdata,2)) = subdata(1,:);

% Only allow 4 plots on any given page. This is current not accounted for
for j=1:size(subdata,2)
    
    % Evenly distribute the plots in rows of 2
    subplot(ceil(nbr_subplots/2),2,j);
    
    hold off;
    
    for k = 2:(size(subdata,1))
        % Make subplots that overlay on one another - function requires
        % figure's handle for plotting in the correct window
        RJ_make_subplot(subdata{k,j}, fig_hdl);
        
        % Only enable holding after first graph is placed. Its ok to
        % iterate it every time.
        hold on;
    end
    
    % Remove underscores in title and replace with a space
    plot_headings(j) = strrep(plot_headings(j), '_', ' ');
    
    % Enable the plot's grid and place the title & axis labels
    grid on;
    title( plot_headings(j), 'fontweight', 'bold', 'fontsize', 16);
    xlabel('Sample', 'fontweight', 'bold');
    ylabel('Value', 'fontweight', 'bold');
    
    % Unset holding for the next iteration's plot
    hold off
    
end

end


function RJ_make_subplot( subplot_data, fig )
    %figure(fig);
    stairs(1:size(subplot_data,1), subplot_data, 'linewidth', 1, 'color', 'green');
    plot(1:size(subplot_data,1), subplot_data, 'linewidth', 1, 'color', 'blue');
end
