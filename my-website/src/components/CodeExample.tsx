import React from 'react';
import clsx from 'clsx';
import styles from './CodeExample.module.css';

// Define the props interface
interface CodeExampleProps {
  title?: string;
  language: string;
  code: string;
  description?: string;
  fileName?: string;
}

// CodeExample component
const CodeExample: React.FC<CodeExampleProps> = ({ 
  title, 
  language, 
  code, 
  description, 
  fileName 
}) => {
  return (
    <div className={clsx('margin-vert--md', styles.codeExample)}>
      {title && <h4 className={styles.title}>{title}</h4>}
      
      {fileName && (
        <div className={styles.fileName}>
          <span className={styles.fileNameText}>{fileName}</span>
        </div>
      )}
      
      <div className={styles.codeContainer}>
        <pre className={styles.pre}>
          <code className={`language-${language}`}>
            {code}
          </code>
        </pre>
      </div>
      
      {description && (
        <div className={styles.description}>
          {description}
        </div>
      )}
    </div>
  );
};

export default CodeExample;