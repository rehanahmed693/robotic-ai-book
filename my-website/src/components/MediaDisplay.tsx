import React from 'react';
import clsx from 'clsx';
import styles from './MediaDisplay.module.css';

// Define the props interface
interface MediaDisplayProps {
  type: 'image' | 'diagram' | 'screenshot' | 'video';
  src: string;
  alt: string;
  caption?: string;
  description?: string;
  width?: string;
  height?: string;
}

// MediaDisplay component
const MediaDisplay: React.FC<MediaDisplayProps> = ({ 
  type, 
  src, 
  alt, 
  caption, 
  description,
  width = '100%',
  height = 'auto'
}) => {
  const mediaStyle = {
    width,
    height
  };

  return (
    <div className={clsx('margin-vert--md', styles.mediaDisplay)}>
      <div className={styles.mediaContainer}>
        {type === 'video' ? (
          <video 
            src={src} 
            alt={alt} 
            style={mediaStyle} 
            controls 
            className={styles.mediaElement}
          />
        ) : (
          <img 
            src={src} 
            alt={alt} 
            style={mediaStyle} 
            className={styles.mediaElement}
          />
        )}
      </div>
      
      {(caption || description) && (
        <div className={styles.captionContainer}>
          {caption && <div className={styles.caption}><strong>{caption}</strong></div>}
          {description && <div className={styles.description}>{description}</div>}
        </div>
      )}
    </div>
  );
};

export default MediaDisplay;