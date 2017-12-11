function im = extractImageFromMsg(im_msg)
im = reshape(im_msg.Data, im_msg.Height, im_msg.Width);
end